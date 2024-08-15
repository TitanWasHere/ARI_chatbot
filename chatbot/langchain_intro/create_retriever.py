import dotenv
from langchain.document_loaders.csv_loader import CSVLoader
from langchain_community.vectorstores import Chroma
from langchain_openai import AzureOpenAIEmbeddings
from langchain.embeddings import HuggingFaceEmbeddings
import os

ARI_CSV_PATH = "../data/responses.csv"
ARI_CHROMA_PATH = "ari_chroma.json"
dotenv.load_dotenv()

# Load the ARI dataset
loader = CSVLoader(file_path=ARI_CSV_PATH, source_column="text_if_error")
ari_dataset = loader.load()

model_name = "sentence-transformers/all-mpnet-base-v2"
model_kwargs = {'device': 'cpu'}
encode_kwargs = {'normalize_embeddings': False}
hf = HuggingFaceEmbeddings(
    model_name=model_name,
    model_kwargs=model_kwargs,
    encode_kwargs=encode_kwargs
)

ari_vector_db = Chroma.from_documents(
    ari_dataset, hf, persist_directory=ARI_CHROMA_PATH
)

