from django.conf import settings
from pymongo import MongoClient

_client = MongoClient(settings.MONGO_URI)
_db = _client[settings.MONGO_DB_NAME]

users_collection = _db[settings.MONGO_USERS_COLLECTION]
