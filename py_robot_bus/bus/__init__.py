import pprint
import time
import json
from pydantic import BaseModel
from pydantic_core import from_json

DEBUG = False


class Bus:
    store = {}
    channels = {}

    @staticmethod
    def get_bus(cls) -> str:
        return Bus.channels[type(cls).__name__]

    @classmethod
    def register(cls, channel_name: str):
        def decorator(fn):
            if fn.__name__ in cls.store:
                raise Exception(
                    f'Two bus event/command class shared same name "{fn.__name__}"'
                )
            cls.store[fn.__name__] = fn
            cls.channels[fn.__name__] = channel_name
            return fn

        return decorator

    @staticmethod
    def _serialize(cls: BaseModel) -> str:
        return json.dumps({"_name_": type(cls).__name__, "_data_": cls.dict()})

    @classmethod
    def _unserialize(cls, serialized: dict) -> BaseModel | None:
        if serialized["type"] != "message":
            return None
        wrapper = from_json(serialized["data"])
        try:
            return cls.store[wrapper["_name_"]].model_validate(wrapper["_data_"])
        except KeyError:
            raise Exception(f'Class {wrapper['_name_']} is not registered in the bus')

    @classmethod
    def publish_messages(cls, redis, message):
        bus = Bus.get_bus(message)
        serialized = cls._serialize(message)
        if DEBUG:
            pprint.pprint(f"PUBLISHED {time.time()} {bus}: {serialized}")
        redis.publish(bus, serialized)

    @classmethod
    def receive_messages(cls, pubsub):
        message = pubsub.get_message()
        while message:
            if DEBUG:
                pprint.pprint(f"RECEIVED {time.time()} {message}")
            yield cls._unserialize(message)
            message = pubsub.get_message()
