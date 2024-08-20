from dotenv import load_dotenv
import os
from langchain_openai import AzureChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain_community.document_loaders import JSONLoader
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.vectorstores.faiss import FAISS
from langchain_huggingface import HuggingFaceEmbeddings
from langchain.chains import create_retrieval_chain
import json 

load_dotenv()

# Load JSON
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
    deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
    api_key = os.getenv("AZURE_OPENAI_API_KEY")
    azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
    api_version = "2024-02-01"

    model = AzureChatOpenAI(
        api_version = api_version,
        azure_deployment = deployment_name,
        azure_endpoint= azure_endpoint,
        api_key= api_key
    )

    # Define the prompt template
    prompt = ChatPromptTemplate.from_template("""
    Rispondi alla seguente domanda dell'utente: (sapendo che il documento è formato da coppie (topic, lista di keywords associate al topic)")
    Contesto: {context} 
    Domanda: {input}                                          
    """)
    chain = create_stuff_documents_chain(
        llm = model, 
        prompt = prompt
    )

    retriever = vectorStore.as_retriever(k=10)
    retrieval_chain = create_retrieval_chain(
        retriever,
        chain
    )

    return retrieval_chain

vectorStore = create_vector(texts)
chain = create_chain(vectorStore)


# chain = prompt | model
phrase = "Esempio di frase, Qual è il tuo compito?"
response = chain.invoke({"input": f"Sapendo che i topic sono {' '.join(topic_list)}, a quale topic appartiene di più la frase '{phrase}', inoltre mi dici a quali parole chiave è associata questa categoria? Rispondi solamente con 'categoria; keyword1, keyword2...' "})
print(response["answer"])
