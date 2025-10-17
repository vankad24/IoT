import time
import argparse
import sys
import random
from paho.mqtt.client import Client
from paho.mqtt.enums import CallbackAPIVersion
import serial

PORT='COM8'
MQTT_BROKER = 'broker.emqx.io'
LUM_TOPIC = 'laboratory/greenhouse/luminosity'
SYS_TOPIC = 'laboratory/greenhouse/system'

def main(port, baud=9600):
    client = Client(callback_api_version=CallbackAPIVersion.VERSION2)
    client.connect(MQTT_BROKER)
    client.loop_start()
    print('Connected to MQTT')

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)

    ser.write(b'p')
    time.sleep(0.2)
    ser.write(b's')

    start_time = time.time()
    try:
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                if line.startswith('SENSOR_VALUE:'):
                    val = line.split(':', 1)[1]
                    client.publish(LUM_TOPIC, val, qos=1)
                    print(f"Опубликовано {val} → {LUM_TOPIC}")
                else:
                    client.publish(SYS_TOPIC, f"PC1_received:{line}", qos=1)
                    print(f"Служебное: {line}")

            time.sleep(0.05)
    except KeyboardInterrupt:
        print('Exit')
    finally:
        ser.close()
        client.loop_stop()
        client.disconnect()

if __name__ == '__main__':    
    main(PORT)