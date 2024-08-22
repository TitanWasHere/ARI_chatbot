from openai import AzureOpenAI
import streamlit as st
import os
import dotenv
import json
import speech_recognition as sr
from langchain_openai import AzureChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_community.document_loaders import JSONLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.vectorstores.faiss import FAISS
from langchain_huggingface import HuggingFaceEmbeddings
from langchain.chains import create_retrieval_chain
from langchain.chains import create_history_aware_retriever

# Caricamento delle variabili d'ambiente
dotenv.load_dotenv()

# Inizializzazione Streamlit
st.title("Chat with ARI 💬")

# Inizializzazione dell'API di Azure OpenAI
deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
api_key = os.getenv("AZURE_OPENAI_API_KEY")
azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
api_version = "2024-02-01"
client = AzureOpenAI(
    api_version=api_version,
    azure_deployment=deployment_name,
    azure_endpoint=azure_endpoint,
    api_key=api_key
)

if "openai_model" not in st.session_state:
    st.session_state["openai_model"] = "gpt-4o"

if "messages" not in st.session_state:
    st.session_state.messages = []

# Caricamento dei file JSON
DIRECTORY_PATH = "../data/"
TOPICS_FILE = "topics.json"
POI_FILE = "points_of_interest.json"

# Caricamento dei file JSON
topics = {}
topic_list = []
poi = {}

with open(DIRECTORY_PATH + TOPICS_FILE) as f:
    topics = json.load(f)


with open(DIRECTORY_PATH + POI_FILE) as f:
    poi = json.load(f)


# Unire i dati dei due file in un'unica lista di documenti
loader_topics = JSONLoader(
    file_path=DIRECTORY_PATH + TOPICS_FILE,
    text_content=False,
    jq_schema="."
)

loader_poi = JSONLoader(
    file_path=DIRECTORY_PATH + POI_FILE,
    text_content=False,
    jq_schema="."
)

data_topics = loader_topics.load()
data_poi = loader_poi.load()
data = data_topics + data_poi

splitter = RecursiveCharacterTextSplitter(chunk_size=200, chunk_overlap=20)
texts = splitter.split_documents(data)

def create_vector(docs):
    embedding = HuggingFaceEmbeddings(
        model_name="sentence-transformers/all-mpnet-base-v2"
    )
    vectorStore = FAISS.from_documents(docs, embedding=embedding)
    return vectorStore

def create_chain(vectorStore, topics, poi):
    model = AzureChatOpenAI(
        api_version=api_version,
        azure_deployment=deployment_name,
        azure_endpoint=azure_endpoint,
        api_key=api_key
    )

    # Definizione del template del prompt
    prompt = ChatPromptTemplate.from_template(
    """
    I valori di {topics} sono il nome della categoria con associata la descrizione e le parole chiave associate a quella categoria. 
    I valori di {poi} sono i punti di interesse in cui vogliamo andare, 
    i valori sono il nome del punto di ineteresse con associate le parole chiave di tale, il suo nome del file .wav associato e come viene chiamato. 
    Quando l'utente ti fa una domanda, capisci a che topic si fa riferimento, se non è nessun topic allora rispondi generalmente e rispondi con il nome del topic. 
    Se la categoria è \"goto\" allora dimmi il punto di interesse più simile associato altrimenti non dire nulla, per farlo dimmi il nome del punto di interesse dalla lista,
    inoltre se non è esattamente chiaro a quale punto di interesse vuole andare, chiedi una conferma fra quelli disponibili usando il loro nome parlato
    Se il punto di interesse viene confermato allora rispondi con "vado a 'nome punto di interesse'.

    Contesto: {context} 
    Domanda: {input}                                          
    """)
    
    #prompt.format(string=string)

    chain = create_stuff_documents_chain(
        llm=model, 
        prompt=prompt,
    )

    retriever = vectorStore.as_retriever(k=10)
    retrieval_chain = create_retrieval_chain(
        retriever,
        chain
    )

    return retrieval_chain

vectorStore = create_vector(texts)
topics_str = json.dumps(topics)
poi_str = json.dumps(poi)
chain = create_chain(vectorStore, topics_str, poi_str)

# Visualizzazione dei messaggi in Streamlit
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

# Funzione per la trascrizione vocale
def recognize_speech_from_mic():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        st.info("Ascoltando...")
        audio = recognizer.listen(source)
        st.info("Elaborando...")
        try:
            return recognizer.recognize_google(audio, language="it-IT")
        except sr.RequestError:
            st.error("Errore di connessione all'API di riconoscimento vocale.")
            return ""
        except sr.UnknownValueError:
            st.warning("Non ho capito cosa hai detto. Per favore riprova.")
            return ""

# Inizializzazione dei messaggi nella sessione di Streamlit
if "messages" not in st.session_state:
    st.session_state.messages = []

# Funzione per costruire il contesto della conversazione
def build_context(messages):
    context = ""
    for msg in messages:
        context += f"{msg['role']}: {msg['content']}\n"
    return context

# Visualizzazione dei messaggi in Streamlit
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])


# Gestione dell'input vocale
if st.button("🎙️"):
    speech_text = recognize_speech_from_mic()
    if speech_text:
        st.session_state.messages.append({"role": "user", "content": speech_text})
        with st.chat_message("user"):
            st.markdown(speech_text)

        context = build_context(st.session_state.messages)

        with st.chat_message("assistant"):
            # Utilizzo di chain.invoke per ottenere la risposta
            response = chain.invoke({
                "input": f"{speech_text}",
                "poi": poi_str,
                "topics": topics_str
            })
            answer = response['answer']

            st.markdown(answer)
            st.session_state.messages.append({"role": "assistant", "content": answer})

# Gestione dell'input dell'utente da tastiera
if prompt := st.chat_input("What is up?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    context = build_context(st.session_state.messages)

    with st.chat_message("assistant"):
        # Utilizzo di chain.invoke per ottenere la risposta
        response = chain.invoke({
            "input": f"{prompt}",
            "poi": poi_str,
            "topics": topics_str
            #string: string
        })
        answer = response['answer']

        st.markdown(answer)
        st.session_state.messages.append({"role": "assistant", "content": answer})
