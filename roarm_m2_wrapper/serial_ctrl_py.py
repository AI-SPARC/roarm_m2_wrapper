import rclpy
from rclpy.node import Node

import serial
import threading
import json
import sys

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray

import json
import serial

ser = serial.Serial("/dev/ttyUSB0",115200)

def read_serial(publisher):
    while True:
        try:
            data = ser.readline().decode('utf-8').strip()
            if '"T":1051' in data:
                # Remover espaços em branco adicionais que podem estar presentes
                data = data.strip()

                print("Received JSON:", data)
                json_data = json.loads(data)
                print("Keys in JSON:", json_data.keys())
                
                if all(key in json_data for key in ['b', 's', 'e', 't', 'torB', 'torS', 'torE', 'torH']):
                    joint_state = JointState()
                    joint_state.name = ['base_to_L1', 'L1_to_L2', 'L2_to_L3', 'L3_to_L4']
                    
                    # Atribuindo position diretamente como listas de valores float
                    joint_state.position = [
                        json_data['b'],
                        json_data['s'],
                        json_data['e'],
                        json_data['t']
                    ]
                    
                    joint_state.effort = [
                        float(json_data['torB']),
                        float(json_data['torS']),
                        float(json_data['torE']),
                        float(json_data['torH'])
                    ]
                    
                    publisher.publish(joint_state)
                    print(f"Published: {joint_state}")
                else:
                    print("Incomplete JSON data received:", json_data)
        except json.JSONDecodeError as e:
            print("Failed to decode JSON:", e)
        except ValueError as e:
            print("Failed to convert value to float:", e)
        except Exception as e:
            print("Error processing JSON:", e)


class MinimalSubscriber(Node):
    def __init__(self, port):
        super().__init__('roarm_m2_wrapper')
        self.position = []
        self.subscription = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.subscription_pose = self.create_subscription(PoseArray, 'objects_position', self.listener_position, 10)
        self.subscription

        self.publisher = self.create_publisher(JointState, 'position', 10)

        
        global ser
        ser = serial.Serial(port, baudrate=115200, dsrdtr=None)
        ser.setRTS(False)
        ser.setDTR(False)

        serial_recv_thread = threading.Thread(target=read_serial, args=(self.publisher,))
        serial_recv_thread.daemon = True
        serial_recv_thread.start()


    
    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            getPos = int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)
            return getPos


    def listener_position(self, msg):
        if msg.poses:
            x = msg.poses[0].position.y + 250.0
            if msg.poses[0].position.x >= 180.0:
                y = msg.poses[0].position.x - 210.0
            else:
                y = (210.0 - msg.poses[0].position.x)* -1.0
            #ajustar manualmente amanhã
            z = 188.0
            t = 3.14

            data = json.dumps({"T": 1041, "x": x, "y": y, "z": z, "t": t}) + "\n"
            ser.write(data.encode() + b'\n')
            print(data)

    def listener_callback(self, msg):
        a = msg.position
        data = json.dumps({'T':102,'base':a[0],'shoulder':a[1],'elbow':a[2],'hand':a[3]+3.1415926,'spd':0,'acc':0}) + "\n"
        # ser.write(data.encode())
        # command = json.dumps({'T': 105}) + "\n"
        # ser.write(command.encode() + b'\n')
        # print(data)


def main(args=None):
    port = '/dev/ttyUSB0'

    rclpy.init(args=sys.argv)
    minimal_subscriber = MinimalSubscriber(port)
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
