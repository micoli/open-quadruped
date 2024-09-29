import redis

redis_client = redis.StrictRedis(host='localhost', port=6379, db=0)
channel = 'robot'
while True:
    message = input("Enter a message: ")
    redis_client.publish(channel, message)