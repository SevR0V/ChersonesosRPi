import json
import paho.mqtt.client as mqtt

def_drone_data = {"position" : {
                    "point":{
                        "lat" : None,
                        "lon" : None,
                        "height": None,
                        },
                    "direction": None,
                    "speed": None
              },
              "battery level": None,
              "heading" : None,
              "status" : "waiting",
              "mission_id" : None,
              "command_id" : None,
              "use_ai_video": None,
              "aim": None,
              "use_hand_mode": None
              }

### переписать под новый формат ###
def_mission_data = {"missionId" : None,
                "missionType" : None,
                "points": [{
                    "lat": None,
                    "long": None,
                    "height": None,
                }],
                "speed": None,
                "rec_video?": None,
                "conf?": None,
                "miss_time?": None,
                "targetObjects?": None,
                "realtime_stream?": None,
                }
###-----------------------------###

class wheelies_mqtt_client:
    def __init__(self, droneId, host, port, username, password):
        self.__droneId = droneId
        self.__host = host
        self.__port = port
        self.__username = username
        self.__password = password

        self.__drone_data = def_drone_data
        self.__mission_data = def_mission_data

        self.__drone_data_topic = str(self.__droneId) + "_data"
        self.__mission_topic = str(self.__droneId) + "_get_mission"

        self.__mqtt_client = mqtt.Client()
        self.__mqtt_client.on_connect = self.__on_connect
        self.__mqtt_client.on_message = self.__on_message

        self.__data_acc_flag = False
    
    def connect(self):        
        self.__mqtt_client.username_pw_set(self.__username, self.__password)
        self.__mqtt_client.connect(self.__host, self.__port, 60)

    def start_loop(self):
        self.__mqtt_client.loop_start()

    def __on_connect(self, client, userdata, flags, rc):
        print("Connected to MQTT broker with result code: " + str(rc))
        self.__mqtt_client.subscribe(self.__mission_topic)
    
    def __on_message(self, client, userdata, msg):        
        data = None
        received_msg = msg.payload.decode()
        try:
            data = json.loads(received_msg)
        except Exception as ex:
            print("JSON parsing error")
            print(ex)
        if data is None:
            return
        self.__mission_data = data
        print("Received message")
        print(self.__mission_data)
        self.__data_acc_flag = True
    
    def __publish(self):
        self.__mqtt_client.publish(self.__drone_data_topic, json.dumps(self.__drone_data), qos= 0)

    def is_connected(self):
        return self.__mqtt_client.is_connected()

    def is_new_data(self):
        return self.__data_acc_flag

    def send_drone_data(self, heading = None, depth = None, speed = None, bat_level = None, 
                        status = "waiting", mission_id = None, direction = None,
                        use_ai_video = None, use_hand_mode = None, aim = None,
                        lon = None, lat = None, command_id = None):
        self.__drone_data["position"]["point"]["lon"] = lon
        self.__drone_data["position"]["point"]["lat"] = lat
        self.__drone_data["position"]["point"]["height"] = depth
        self.__drone_data["position"]["direction"] = direction
        self.__drone_data["position"]["speed"] = speed
        self.__drone_data["battery level"] = bat_level
        self.__drone_data["heading"] = heading       
        self.__drone_data["status"] = status
        self.__drone_data["mission_id"] = mission_id
        self.__drone_data["command_id"] = command_id
        self.__drone_data["use_ai_video"] = use_ai_video        
        self.__drone_data["aim"] = aim
        self.__drone_data["use_hand_mode"] = use_hand_mode
        self.__publish()