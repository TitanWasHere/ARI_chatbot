#!/usr/bin/env python

import rospy
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
from std_msgs.msg import String

class Chatbot:
    def __init__(self):
        dotenv.load_dotenv()
        self.SESSION_ID = "ARI1"
        self.DEPLOYMENT_NAME = os.getenv("DEPLOYMENT_NAME_GPT4o")
        self.API_KEY = os.getenv("AZURE_OPENAI_API_KEY")
        self.AZURE_ENDPOINT = os.getenv("AZURE_OPENAI_ENDPOINT")
        self.API_VERSION = "2024-02-01"
        self.DIRECTORY_PATH = "../data/"
        self.TOPICS_FILE = "topics.json"
        self.RESPONSES_FILE = "responses.json"
        self.POI_FILE = "points_of_interest.json"
        self.topics = []
        self.points_of_interest = []
        self.chat_history = []
        self.openai_model = "gpt-4o"
        self.messages = []

        self.load_topics()
        self.load_points_of_interest()
        docs = self.get_documents()
        self.vectorStore = self.create_db(docs)
        self.chain = self.create_chain(self.vectorStore)
        
        self.resp = ""
        # Rospy subscriber and publisher (topic)
        self.sub = rospy.Subscriber("/process_gpt", String, self.chatbot_msgs)
        self.pub = rospy.Publisher("/chatbot_response", String, queue_size=10)



    def chatbot_msgs(self, req):
        question = req.data
        response = self.chain.invoke({
            "input": question,
            "chat_history": self.chat_history,
            "topics": self.topics,
            "points_of_interest": self.points_of_interest
        })
        self.resp = response['answer']
        self.chat_history.append(HumanMessage(content=question))
        self.chat_history.append(AIMessage(content=self.resp))

        self.pub.publish(self.resp)


        

    def load_topics(self):
        with open("../data/topics.json") as f:
            chat_prompt = json.load(f)
            self.topics = chat_prompt.keys()

    def load_points_of_interest(self):
        with open("../data/points_of_interest.json") as f:
            chat_prompt = json.load(f)
            self.points_of_interest = chat_prompt.keys()

    def get_documents(self):
        loader = JSONLoader(
            file_path = self.DIRECTORY_PATH + self.TOPICS_FILE,
            jq_schema=".",
            text_content=False
        )
        docs1 = loader.load()

        loader2 = JSONLoader(
            file_path = self.DIRECTORY_PATH + self.POI_FILE,
            jq_schema=".",
            text_content=False
        )
        docs2 = loader2.load()

        loader3 = JSONLoader(
            file_path = self.DIRECTORY_PATH + self.RESPONSES_FILE,
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
        
    def create_db(self, docs):

        embedding = HuggingFaceEmbeddings(
            model_name="sentence-transformers/all-mpnet-base-v2"
        )
        vectorStore = FAISS.from_documents(docs, embedding=embedding)
        return vectorStore

    def create_chain(self, vectorStore):
        model = AzureChatOpenAI(
            api_version=self.API_VERSION,
            azure_deployment=self.DEPLOYMENT_NAME,
            azure_endpoint=self.AZURE_ENDPOINT,
            api_key=self.API_KEY
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
    

if __name__ == "__main__":
    try:
        rospy.init_node("chatbot", disable_signals=True)
        chatbot = Chatbot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
