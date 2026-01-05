# authapp/mongo_devices.py
from pymongo import MongoClient
from django.conf import settings

client = MongoClient(settings.MONGO_URI)
db = client["robot_control"]

user_devices_collection = db["user_devices"]
