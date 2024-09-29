import redis


def create_redis(subscribed_channel=None):
    redis_client = redis.StrictRedis(host="localhost", port=6379, db=0)
    if subscribed_channel:
        pubsub = redis_client.pubsub(ignore_subscribe_messages=True)
        for channel in subscribed_channel:
            pubsub.subscribe(channel)
        return redis_client, pubsub
    return redis_client
