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

# Caricamento del file JSON
DIRECTORY_PATH = "../data/"
TOPICS_FILE = "topics.json"
topics = {}
topic_list = []

with open(DIRECTORY_PATH + TOPICS_FILE) as f:
    topics = json.load(f)
    topic_list = topics.keys()

loader = JSONLoader(
    file_path=DIRECTORY_PATH + TOPICS_FILE,
    text_content=False,
    jq_schema=".[]"
)

data = loader.load()
splitter = RecursiveCharacterTextSplitter(chunk_size=200, chunk_overlap=20)
texts = splitter.split_documents(data)

def create_vector(docs):
    embedding = HuggingFaceEmbeddings(
        model_name="sentence-transformers/all-mpnet-base-v2"
    )
    vectorStore = FAISS.from_documents(docs, embedding=embedding)
    return vectorStore

def create_chain(vectorStore):
    model = AzureChatOpenAI(
        api_version=api_version,
        azure_deployment=deployment_name,
        azure_endpoint=azure_endpoint,
        api_key=api_key
    )

    # Definizione del template del prompt
    prompt = ChatPromptTemplate.from_template("""
    Rispondi alla seguente domanda dell'utente (sapendo che il documento è formato da coppie (topic, lista di keywords associate al topic)):
    Contesto: {context} 
    Domanda: {input}                                          
    """)

    chain = create_stuff_documents_chain(
        llm=model, 
        prompt=prompt
    )

    retriever = vectorStore.as_retriever(k=10)
    retrieval_chain = create_retrieval_chain(
        retriever,
        chain
    )

    return retrieval_chain

vectorStore = create_vector(texts)
chain = create_chain(vectorStore)

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

# Gestione dell'input vocale
if st.button("🎙️"):
    speech_text = recognize_speech_from_mic()
    if speech_text:
        st.session_state.messages.append({"role": "user", "content": speech_text})
        with st.chat_message("user"):
            st.markdown(speech_text)

        with st.chat_message("assistant"):
            # Utilizzo di chain.invoke per ottenere la risposta
            response = chain.invoke({
                "input": f"Sapendo che i topic sono {' '.join(topic_list)}, a quale topic appartiene di più la frase '{speech_text}', inoltre mi dici a quali parole chiave è associata questa categoria? Rispondi solamente con 'categoria; keyword1, keyword2...'"
            })
            answer = response['answer']

            st.markdown(answer)
            st.session_state.messages.append({"role": "assistant", "content": answer})

# Gestione dell'input dell'utente da tastiera
if prompt := st.chat_input("What is up?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    with st.chat_message("assistant"):
        # Utilizzo di chain.invoke per ottenere la risposta
        response = chain.invoke({
            "input": f"Sapendo che i topic sono {' '.join(topic_list)}, a quale topic appartiene di più la frase '{prompt}', inoltre mi dici a quali parole chiave è associata questa categoria? Rispondi solamente con 'categoria; keyword1, keyword2...'"
        })
        answer = response['answer']

        st.markdown(answer)
        st.session_state.messages.append({"role": "assistant", "content": answer})
