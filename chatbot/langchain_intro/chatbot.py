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
dotenv.load_dotenv()

# Definisci il template per la risposta
review_template_str = """Il tuo compito è quello di rispondere alle domande delle persone
riguardanti il tuo lavoro e alle informazioni che hai memorizzate. Sii il più
dettagliato possibile ma non esprimere informazioni che non siano nel contesto. Se non
sai la risposta basta dire che non la sai. Cerca di rispondere con frasi non troppo lunghe ma comunque complete.

{context}
"""

# Definisci il template per la risposta, dando del contesto
review_system_prompt = SystemMessagePromptTemplate(
    prompt=PromptTemplate(
        input_variables=["context"],
        template=review_template_str,
    )
)

# Definisci il template per la domanda dell'utente
review_human_prompt = HumanMessagePromptTemplate(
    prompt=PromptTemplate(
        input_variables=["question"],
        template=review_template_str,
    )
)

messages = [review_system_prompt, review_human_prompt]

review_prompt_template = ChatPromptTemplate(
    input_variables=["context", "question"],
    messages=messages,
)

# deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4-32k")
# deployment_name = os.getenv("DEPLOYMENT_NAME_GPT35-TURBO")
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

output_parser = StrOutputParser() # Permette di analizzare solo la stringa di risposta
review_chain = review_prompt_template | chat | output_parser
