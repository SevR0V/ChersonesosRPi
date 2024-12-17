from wheelies import wheelies_mqtt_xfer_client
import time
import json
import random
# control_data = {"missionId" : None,
#                 "missionType" : None,
#                 "points": [{
#                 "lat": None,
#                 "long": None,
#                 "height": None,
#                 }],
#                 "speed": None,
#                 "rec_video?": None,
#                 "conf?": None,
#                 "miss_time?": None,
#                 "targetObjects?": None,
#                 "realtime_stream?": None,
#                 }
# data = json.dumps(control_data)
# print(control_data)
# exit

#WClient = wheelies_mqtt_xfer_client(10, "dev.wheelies.ru", 1883, "rabbitmq_dev_mqtt", "Ms4oQxQfeqSQUg")
WClient = wheelies_mqtt_xfer_client(10, "m5.wqtt.ru", 12388, "u_477FPB", "7FOT2qvH")
WClient.connect()
WClient.start_loop()
time.sleep(2)
if not WClient.is_connected():
    exit()
while(1):
    heading = random.random() * 360
    WClient.send_drone_data(heading= heading)
    time.sleep(1)