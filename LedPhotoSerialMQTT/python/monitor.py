import time
from paho.mqtt.client import Client, MQTTMessage
from paho.mqtt.enums import CallbackAPIVersion

BROKER = 'broker.emqx.io'
LUM_TOPIC = 'laboratory/greenhouse/luminosity'
SYS_TOPIC = 'laboratory/greenhouse/system'

TOPICS = [
    LUM_TOPIC,
    SYS_TOPIC
]

def on_connect(client, userdata, flags, rc, properties=None):
    print('Monitor connected' if rc == 0 else f'Monitor connect failed {rc}')
    for t in TOPICS:
        client.subscribe(t, qos=1)

def on_message(client, userdata, msg: MQTTMessage):
    print(f"[{msg.topic}] {msg.payload.decode('utf-8')}")

if __name__ == '__main__':
    c = Client(callback_api_version=CallbackAPIVersion.VERSION2)
    c.on_connect = on_connect
    c.on_message = on_message
    c.connect(BROKER)
    c.loop_forever()