from wheelies import wheelies_mqtt_client
import time

wheelies_client = wheelies_mqtt_client(10, "dev.wheelies.ru", 1883, "rabbitmq_dev_mqtt", "Ms4oQxQfeqSQUg")
# WClient = wheelies_mqtt_xfer_client(10, "m5.wqtt.ru", 12388, "u_477FPB", "7FOT2qvH")
wheelies_client.connect()
wheelies_client.start_loop()
time.sleep(2)
if not wheelies_client.is_connected():
    exit()
while(1):
    wheelies_client.send_drone_data()
    time.sleep(1)