import streamlit as st
from langchain_openai import AzureChatOpenAI
import dotenv
import os

SESSION_ID = "ARI1"

# Caricamento delle variabili d'ambiente
dotenv.load_dotenv()

# Inizializzazione Streamlit
st.title("Chat with ARI 💬")

# Inizializzazione dell'API di Azure OpenAI
deployment_name = os.getenv("DEPLOYMENT_NAME_GPT4o")
api_key = os.getenv("AZURE_OPENAI_API_KEY")
azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
api_version = "2024-02-01"

client = AzureChatOpenAI(
    api_version=api_version,
    azure_deployment=deployment_name,
    azure_endpoint=azure_endpoint,
    api_key=api_key,
)


# Set a default model
if "openai_model" not in st.session_state:
    st.session_state["openai_model"] = "gpt-3.5-turbo"

# Initialize chat history
if "messages" not in st.session_state:
    st.session_state.messages = []

# Display chat messages from history on app rerun
for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

# Gestione dell'input dell'utente da tastiera
if prompt := st.chat_input("What is up?"):
    st.session_state.messages.append({"role": "user", "content": prompt})
    with st.chat_message("user"):
        st.markdown(prompt)

    context = build_context(st.session_state.messages)

    with st.chat_message("assistant"):
        response = chain.invoke(
            {
                "input": f"{prompt}",
                "poi": poi_str,
                "topics": topics_str,
                "history": get_session_history(SESSION_ID).messages  # Passa la cronologia qui
            },
            config={
                "configurable": {"session_id": SESSION_ID}
            }
        )
        answer = response['answer']

        st.session_state.messages.append({"role": "assistant", "content": answer})
