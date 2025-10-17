import time
import argparse
import random
from paho.mqtt.client import Client, MQTTMessage
from paho.mqtt.enums import CallbackAPIVersion
import serial

PORT="COM7"
MQTT_BROKER = 'broker.emqx.io'
LUM_TOPIC = 'laboratory/greenhouse/luminosity'
SYS_TOPIC = 'laboratory/greenhouse/system'

THRESHOLD = 24

class PC2:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, 9600, timeout=1)
        time.sleep(2)
        self.client = Client(callback_api_version=CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(MQTT_BROKER)
        self.client.loop_start()
        self.client.subscribe([(LUM_TOPIC, 1)])

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print('Connected to MQTT')
        else:
            print('MQTT connect failed', rc)

    def on_message(self, client, userdata, msg: MQTTMessage):
        payload = msg.payload.decode('utf-8')
        topic = msg.topic
        print(f"MQTT {topic} → {payload}")

        if topic == LUM_TOPIC:
            try:
                val = int(payload)
            except ValueError:
                return
            if val < THRESHOLD:
                self.send_cmd('u')
                self.client.publish(SYS_TOPIC, 'Led: auto ON', qos=1)
            else:
                self.send_cmd('d')
                self.client.publish(SYS_TOPIC, 'Led: auto OFF', qos=1)

    def send_cmd(self, cmd):
        self.ser.write(cmd.encode())
        print(f"Отправлено MCU: {cmd}")

    def close(self):
        self.client.loop_stop()
        self.client.disconnect()
        self.ser.close()

if __name__ == '__main__':
    pc2 = PC2(PORT)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pc2.close()
        print('Exit')