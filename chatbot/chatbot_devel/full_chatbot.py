import streamlit as st
import os
import dotenv
import json
import speech_recognition as sr
from langchain_openai import AzureChatOpenAI
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_community.document_loaders import JSONLoader, DirectoryLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.vectorstores.faiss import FAISS
from langchain_huggingface import HuggingFaceEmbeddings
from langchain.chains import create_retrieval_chain, create_history_aware_retriever
from langchain_core.runnables.history import RunnableWithMessageHistory
from langchain_core.messages import HumanMessage, AIMessage
from langchain_core.prompts import MessagesPlaceholder
from langchain.chains.history_aware_retriever import create_history_aware_retriever

dotenv.load_dotenv()

SESSION_ID = "ARI1"
DEPLOYMENT_NAME = os.getenv("DEPLOYMENT_NAME_GPT4o")
API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
AZURE_ENDPOINT = os.getenv("AZURE_OPENAI_ENDPOINT")
API_VERSION = "2024-02-01"

DIRECTORY_PATH = "../data/"
TOPICS_FILE = "topics.json"
RESPONSES_FILE = "responses.json"
POI_FILE = "points_of_interest.json"

# Initialize session state for chat history
if "chat_history" not in st.session_state:
    st.session_state.chat_history = []

if "openai_model" not in st.session_state:
    st.session_state["openai_model"] = "gpt-4o"

if "messages" not in st.session_state:
    st.session_state.messages = []

topics = []
points_of_interest = []

st.title("Chat with ARI 💬")

# Load the topics and points of interest
with open("../data/topics.json") as f:
    chat_prompt = json.load(f)
    topics = chat_prompt.keys()

with open("../data/points_of_interest.json") as f:
    chat_prompt = json.load(f)
    points_of_interest = chat_prompt.keys()

# Get documents and create the vector store
def get_documents():

    loader = JSONLoader(
        file_path = DIRECTORY_PATH + TOPICS_FILE,
        jq_schema=".",
        text_content=False
    )
    docs1 = loader.load()

    loader2 = JSONLoader(
        file_path = DIRECTORY_PATH + POI_FILE,
        jq_schema=".",
        text_content=False
    )
    docs2 = loader2.load()

    loader3 = JSONLoader(
        file_path = DIRECTORY_PATH + RESPONSES_FILE,
        jq_schema=".",
        text_content=False
    )
    docs3 = loader3.load()

    docs = docs1 + docs2 + docs3


    splitter = RecursiveCharacterTextSplitter(
        chunk_size=400,
        chunk_overlap=20
    )

    splitDocs = splitter.split_documents(docs)
    return splitDocs

def create_db(docs):
    embedding = HuggingFaceEmbeddings(
        model_name="sentence-transformers/all-mpnet-base-v2"
    )
    vectorStore = FAISS.from_documents(docs, embedding=embedding)
    return vectorStore

def create_chain(vectorStore):
    model = AzureChatOpenAI(
        api_version=API_VERSION,
        azure_deployment=DEPLOYMENT_NAME,
        azure_endpoint=AZURE_ENDPOINT,
        api_key=API_KEY
    )

    prompt_chat = """
    Tu sei ARI, un robot umanoide dell'università di Trento, il cui tuo scopo è quello di interagire con le persone.
    La prima cosa che devi sempre fare è capire a che categoria si riferisce la domanda dell'utente, anche se la domanda viene ripetuta.
    I valori {topics} sono le categorie. Ad esse sono associate la descrizione e le parole chiave associate a quella categoria.
    Inoltre, sono associati anche delle risposte generiche con nome associato "text_if_error", queste risposte sono le risposte associate al topic.
    I valori {poi} sono i punti di interesse in cui vogliamo andare, 
    i valori sono il nome del punto di interesse con associate le parole chiave di tale, il suo nome del file .wav associato e come viene chiamato. 
    Quando l'utente ti fa una domanda, capisci a che topic si fa riferimento, se non si riferisce particolarmente a nessuna categoria, allora dai comunque una risposta, altrimenti inizia la frase con '!topic_'+nome della categoria e rispondi con la risposta associata al topic o in un modo simile ogni volta che si ri presenta il topic. 
    Se la categoria è \"goto\" allora dimmi il punto di interesse più simile associato altrimenti non dire nulla, per farlo dimmi il nome del punto di interesse dalla lista,
    inoltre sia se è chiaro ed è la prima volta che viene chiesto, sia che non è esattamente chiaro a quale punto di interesse vuole andare, chiedi una conferma fra quelli disponibili usando il loro nome parlato, finchè non è esattamente chiaro a quale punto di interesse ci si riferisce chiedi sempre una maggiore conferma più dettagliata
    Se il punto di interesse viene confermato allora rispondi con "vado a 'nome punto di interesse', non rispondere mai con "vado a ..." in altre situazioni.
    
    Basati sul contesto: {context}
    """

    prompt = ChatPromptTemplate.from_messages([
        ("system", prompt_chat),
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}")
    ])

    chain = create_stuff_documents_chain(
        llm=model,
        prompt=prompt
    )

    retriever = vectorStore.as_retriever(search_kwargs={"k": 3})

    retriever_prompt = ChatPromptTemplate.from_messages([
        MessagesPlaceholder(variable_name="chat_history"),
        ("human", "{input}"),
        ("human", "Data la conversazione soprastante, genera una query di ricerca per trovare la risposta più appropriata")
    ])

    history_aware_retriever = create_history_aware_retriever(
        llm=model,
        retriever=retriever,
        prompt=retriever_prompt
    )

    retriever_chain = create_retrieval_chain(
        history_aware_retriever,
        chain
    )

    return retriever_chain



docs = get_documents()
vectorStore = create_db(docs)
chain = create_chain(vectorStore)

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

        with st.chat_message("assistant"):
            response = chain.invoke(
                {
                    "input": f"{speech_text}",
                    "poi": points_of_interest,
                    "topics": topics,
                    "chat_history": st.session_state.chat_history,
                }
            )
            answer = response['answer']
            st.markdown(answer)
            st.session_state.messages.append({"role": "assistant", "content": answer})
            st.session_state.chat_history.append(HumanMessage(content=f"{speech_text}"))
            st.session_state.chat_history.append(AIMessage(content=answer))

# Gestione dell'input testuale
if prompt := st.chat_input("What is up?"):
    st.session_state.messages.append({"role": "user", "content": prompt})

    with st.chat_message("user"):
        st.markdown(prompt)

    with st.chat_message("assistant"):
        response = chain.invoke(
            {
                "input": f"{prompt}",
                "poi": points_of_interest,
                "topics": topics,
                "chat_history": st.session_state.chat_history
            }
        )
        answer = response['answer']
        st.markdown(answer)
        st.session_state.messages.append({"role": "assistant", "content": answer})
        st.session_state.chat_history.append(HumanMessage(content=f"{prompt}"))
        st.session_state.chat_history.append(AIMessage(content=answer))

        # if answer.lower().startswith("vado a"):
        #     st.session_state.chat_history = []

