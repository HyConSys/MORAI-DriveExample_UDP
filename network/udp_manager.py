import numpy as np
from autonomous_driving.vehicle_state import VehicleState
from autonomous_driving.perception.object_info import ObjectInfo
from autonomous_driving.config.config import Config
from .sender import CtrlCmdSender, TrafficLightSender
from .receiver import EgoInfoReceiver, ObjectInfoReceiver, TrafficLightReceiver
import os
import threading
import time
import copy
from math import sqrt


class UdpManager:
    def __init__(self, autonomous_driving, safety_shield):
        self.autonomous_driving = autonomous_driving
        self.safety_shield = safety_shield

        self.config = Config()
        self.config.update_config(os.path.join(os.path.dirname(__file__), 'config.json'))
        self.traffic_light_control = self.config['map']['traffic_light_control']
        self.sampling_time = 1/float(self.config['common']['sampling_rate'])

        self.vehicle_state = None
        self.vehicle_state_lock = threading.Lock()

        self.object_info_list = []
        self.object_info_list_lock = threading.Lock()

        self.traffic_light = []
        self.traffic_light_lock = threading.Lock()

        self.prev_x = 0
        self.prev_y = 0

        self.path_list_x = []
        self.path_list_y = []
        
        self.vehicle_max_steering_data = 36.25

    def execute(self):
        print('start simulation')
        self._set_protocol()
        self._main_loop()

    def _set_protocol(self):
        network = self.config['network']
        # sender
        self.ctrl_cmd_sender = CtrlCmdSender(network['user_ip'], network['ctrl_cmd_host_port'])
        self.traffic_light_sender = TrafficLightSender(network['user_ip'], network['set_traffic_host_port'])

        # receiver
        self.ego_info_receiver = EgoInfoReceiver(
            network['host_ip'], network['ego_info_dst_port'], self._ego_info_callback
        )
        self.object_info_receiver = ObjectInfoReceiver(
            network['host_ip'], network['object_info_dst_port'], self._object_info_callback
        )
        self.traffic_light_receiver = TrafficLightReceiver(
            network['host_ip'], network['get_traffic_dst_port'], self._traffic_light_callback
        )

    def _main_loop(self):

        while True:
            start_time = time.perf_counter()
            compen_time = 0

            # collect needed data
            local_vehicle_state = []
            with self.vehicle_state_lock:
                if self.vehicle_state:
                    local_vehicle_state = copy.deepcopy(self.vehicle_state)
            
            local_object_info_list = []
            with self.object_info_list_lock:
                if self.object_info_list:
                    local_object_info_list = copy.deepcopy(self.object_info_list)

            local_traffic_light = []
            if self.traffic_light:
                local_traffic_light = copy.deepcopy(self.traffic_light)


            if local_vehicle_state:
                
                unschielded_control_input, _ = self.autonomous_driving.execute(local_vehicle_state, local_object_info_list, local_traffic_light)
                
                control_input = self.safety_shield.check_safety(local_vehicle_state, local_object_info_list, unschielded_control_input)

                steering_input = -np.rad2deg(control_input.steering)/self.vehicle_max_steering_data
                self.ctrl_cmd_sender.send_data([control_input.accel, control_input.brake, steering_input])
                print("Control Data:" , [control_input.accel, control_input.brake, steering_input])
                
                end_time = time.perf_counter()
                self._print_info(control_input)
                compen_time = float((end_time - start_time))
                
            if((1/30 - compen_time) > 0):
                time.sleep(1/30 - compen_time)

    def _print_info(self, control_input):
        os.system('cls')
        print('--------------------status-------------------------')
        with self.vehicle_state_lock:
            print(f'x: {self.vehicle_state.position.x:.4f}, y: {self.vehicle_state.position.y:.4f}')
            print(f'velocity: {self.vehicle_state.velocity*3.6:.4f} km/h')
            print(f'heading: {np.rad2deg(self.vehicle_state.yaw):.4f} deg')

        print('--------------------controller-------------------------')
        print(f'accel: {control_input.accel:.4f}')
        print(f'brake: {control_input.brake:.4f}')
        print(f'steering_angle: {-np.rad2deg(control_input.steering):.4f} deg')

        with self.object_info_list_lock:
            if self.object_info_list:
                print('--------------------object-------------------------')
                print(f'object num: {len(self.object_info_list)}')
                for i, object_info in enumerate(self.object_info_list):
                    print(
                        f'#{i} type: {object_info.type}, x: {object_info.position.x:.4f}, '
                        f'y: {object_info.position.y:.4f}, velocity: {object_info.velocity:.4f}'
                    )

        with self.traffic_light_lock:
            if self.traffic_light:
                print('--------------------traffic light-------------------------')
                print(f'traffic index: {self.traffic_light[0]}')
                print(f'traffic status: {self.traffic_light[1]}')

    def _ego_info_callback(self, data):
        with self.vehicle_state_lock:
            if data:
                self.vehicle_state = VehicleState(data[12], data[13], np.deg2rad(data[17]), data[18]/3.6)
                self.vehicle_currenty_steer = data[-1]
            else:
                self.vehicle_state = None

    def _object_info_callback(self, data_list):
        with self.object_info_list_lock:
            if data_list:
                self.object_info_list = [ObjectInfo(data[2], data[3], data[12], data[1], "", data[6], data[7], data[8]) for data in data_list]
            else:
                self.object_info_list = []

    def _traffic_light_callback(self, data):
        with self.traffic_light_lock:
            if data:
                # traffic_control (차량 신호 Green Light(16) 변경)
                if self.traffic_light_control:
                    traffic_light_status = 16
                    self.traffic_light_sender.send_data([data[0], traffic_light_status])
                else:
                    traffic_light_status = data[2]

                self.traffic_light = [data[0], traffic_light_status]
            else:
                self.traffic_light = []
