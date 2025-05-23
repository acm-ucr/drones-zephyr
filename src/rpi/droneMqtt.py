import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "drone/zephyrcommands"

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe(MQTT_TOPIC)

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Message received: {message}")
    if message == "Left":
        print("Turning Left")
    elif message == "Right":
        print("Turning Right")
    

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)

print("Listening on MQTT topic:", MQTT_TOPIC)
client.loop_forever()