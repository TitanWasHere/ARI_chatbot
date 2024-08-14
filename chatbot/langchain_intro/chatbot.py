import dotenv
import os
from langchain_openai import AzureChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage

dotenv.load_dotenv()

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

# messages = [
#     (
#         "system",
#         "You are a helpful translator. Translate the user sentence to French.",
#     ),
#     ("human", "I love programming."),
# ]
# chat.invoke(messages)

