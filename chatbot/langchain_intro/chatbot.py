import dotenv
import os
import openai

dotenv.load_dotenv()

openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_type = "azure"
openai.azure_endpoint = os.getenv("OPENAI_KEY_ENDPOINT")
openai.api_version = "2024-02-01"
deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
# deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4-32k")
# deployment_name = os.getenv("DEPLOYMENT_NAME_GPT35-TURBO")

# print("Sending a test completion job")
# start_phrase = [
#     {"role": "user", "content": "Give me the sum of the first 3 prime numbers"}
# ]
# response = openai.chat.completions.create(model=deployment_name, messages=start_phrase)
# print(response.choices[0].message.content)
