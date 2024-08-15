import dotenv
import os
from langchain_openai import AzureChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage
from langchain.prompts import (
    PromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
    ChatPromptTemplate
)
from langchain_core.output_parsers import StrOutputParser
from langchain_community.vectorstores import Chroma
from langchain_huggingface import HuggingFaceEmbeddings
from langchain.schema.runnable import RunnablePassthrough
from langchain.indexes import VectorstoreIndexCreator
from langchain_community.document_loaders import JSONLoader
import json
from langchain.text_splitter import CharacterTextSplitter
from langchain.chains import RetrievalQA

DIRECTORY_PATH = "../data/"
TOPICS_FILE = "topics.json"

dotenv.load_dotenv()

deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
api_key = os.getenv("AZURE_OPENAI_API_KEY")
azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
api_version = "2024-02-01"

# Initialize the ChatOpenAI object for Azure
chat = AzureChatOpenAI(
    api_version = api_version,
    azure_deployment = deployment_name,
    azure_endpoint= azure_endpoint,
    api_key= api_key
)

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

text_splitter = CharacterTextSplitter(chunk_size=1000, chunk_overlap=0)
texts = text_splitter.split_documents(data)

embeddings = HuggingFaceEmbeddings(
    model_name="sentence-transformers/all-mpnet-base-v2",
    model_kwargs={'device': 'cpu'},
    encode_kwargs={'normalize_embeddings': True}
)

docsearch = Chroma.from_documents(
    texts, embeddings
)

qa = RetrievalQA.from_chain_type(llm=chat, chain_type="stuff", retriever=docsearch.as_retriever())


output = qa.invoke(f"Sapendo che i topic sono {' '.join(topic_list)}, a quale topic appartiene di più la frase 'Qual è il tuo compito?', inoltre mi dici a quali parole chiave è associata questa categoria? Rispondi solamente con 'categoria; keyword1, keyword2...' ")
output = output["result"]
print(output) # Expected output: "topic; key1, key2..."
#print(qa.invoke("Chi è George Washington?"))

