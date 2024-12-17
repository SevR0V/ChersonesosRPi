from wheelies import wheelies_mqtt_xfer_client
import time
import random

WClient = wheelies_mqtt_xfer_client(10, "dev.wheelies.ru", 1883, "rabbitmq_dev_mqtt", "Ms4oQxQfeqSQUg")
# WClient = wheelies_mqtt_xfer_client(10, "m5.wqtt.ru", 12388, "u_477FPB", "7FOT2qvH")
WClient.connect()
WClient.start_loop()
time.sleep(2)
if not WClient.is_connected():
    exit()
while(1):
    WClient.send_drone_data()
    time.sleep(1)