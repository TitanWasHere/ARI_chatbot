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

topics = []
points_of_interest = []

with open("../data/topics.json") as f:
    chat_prompt = json.load(f)
    topics = chat_prompt.keys()
        
with open("../data/points_of_interest.json") as f:
    chat_prompt = json.load(f)
    points_of_interest = chat_prompt.keys()

# Inizializzazione dell'API di Azure OpenAI
deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
api_key = os.getenv("AZURE_OPENAI_API_KEY")
azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
api_version = "2024-02-01"

def get_documents(url):

    loader = JSONLoader(
        file_path = url + "topics.json",
        jq_schema=".",
        text_content=False
    )
    docs1 = loader.load()

    loader2 = JSONLoader(
        file_path = url + "points_of_interest.json",
        jq_schema=".",
        text_content=False
    )
    docs2 = loader2.load()

    docs = docs1 + docs2

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
        api_version=api_version,
        azure_deployment=deployment_name,
        azure_endpoint=azure_endpoint,
        api_key=api_key
    )

    
    
    #    Answer the user question knowing that the topics are: {topics} and the points of interest are: {points_of_interest}

    prompt = ChatPromptTemplate.from_messages([
        ("system", "Answer the user question knowing that the topics are: {topics} and the points of interest are: {points_of_interest} based on the context: {context}"),
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
        ("human", "given the above conversation, generate a search query to look up in order to get information relevant to the conversation")
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

def process_chat(chain, question, chat_history):
    response = chain.invoke({
        "input": question,
        "topics": topics,
        "chat_history": chat_history,
        "points_of_interest": points_of_interest
    })

    return(response["answer"])

if __name__ == "__main__":
    docs = get_documents("../data/")
    vectorStore = create_db(docs)
    chain = create_chain(vectorStore)
    
    chat_history = [
    ]

    while True:
        user_input = input("Tu: ")
        if user_input.lower() == "exit":
            break
        resp = process_chat(chain, user_input, chat_history)
        chat_history.append(HumanMessage(content=user_input))
        chat_history.append(AIMessage(content=resp))
        print("AI;", resp)
