import paho.mqtt.client as mqtt
import json
import time

#Konfiguration
mqtt_broker = "4.184.199.231"
mqtt_port = 1883
topics_to_subscribe = [
    ("devices/mack_multisensor-1_json", 0),
    ("devices/mack_multisensor-2_json", 0),
    ("devices/mack_multiparametersonde-1_json", 0),
    ("hello world", 0)
]
mqtt_username = "admin"
mqtt_password = "hasenpasswort"
thingsboard_host = "20.218.87.198" 
thingsboard_port = 8080
device_token = "gxU5OxAG1GTmN0SkLMxw"


#Callback-Funktion für erfolgreiche Verbindung
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Verbunden mit MQTT Broker mit Code =", rc)
        client.subscribe(topics_to_subscribe)
    else:
        print("Schlechte Verbindung, Fehlercode =", rc)


#Callback-Funktion für den Empfang einer Nachricht
def on_message(client, userdata, msg):
    print(f"Nachricht erhalten: {msg.topic} {str(msg.payload)}")
    tb_client.publish("v1/devices/me/telemetry", msg.payload)
    
mqtt_client = mqtt.Client()
mqtt_client.username_pw_set(mqtt_username, mqtt_password)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker, mqtt_port, 1883)

tb_client = mqtt.Client()
tb_client.username_pw_set(device_token)
tb_client.connect(thingsboard_host, thingsboard_port, 8080)

# Startet den Loop
mqtt_client.loop_start()
tb_client.loop_start()

while True:
    time.sleep(10)