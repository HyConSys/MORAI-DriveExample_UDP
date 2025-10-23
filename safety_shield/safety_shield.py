#!/usr/bin/env python

from .arena_visualizer import ArenaVisualizer
from .pfaces_sym_control.client import pFacesSymControlClient
from .utils import str2list, list2str
from autonomous_driving.localization.point import Point

import threading
import time
import sys
import math
import os
import copy
from numpy import linalg

# insert interace folder of pFaces
if 'PFACES_SDK_ROOT' in os.environ:
    pfaces_interface_path = sys.path.insert(1, os.environ['PFACES_SDK_ROOT'] + "/../interface/python")
else:
    raise Exception('pFaces is not installed correctly.')
from ConfigReader import ConfigReader

class SafetyShield:
    def __init__(self):
        self.client = pFacesSymControlClient("http://127.0.0.1:12345", "pFaces/REST/dictionary/morai_acas")
        self.config_file = "safety_shield\\pfaces_sym_control\\morai_acas.cfg"
        self.config_reader = ConfigReader(self.config_file)

        # state space info
        self.x_lb = str2list(self.config_reader.get_value_string("states.lb"))
        self.x_ub = str2list(self.config_reader.get_value_string("states.ub"))
        self.x_eta = str2list(self.config_reader.get_value_string("states.eta"))

        # arena information
        self.arena_width = self.x_ub[0] - self.x_lb[0]
        self.arena_height = self.x_ub[1] - self.x_lb[1]

        # car's position in pfaces grid (concrete value in meters, relative to state space (AKA arena) base [0,0,0])
        # this shal also serve as intitial position (which becomes intial state if velocity included)
        self.car_pos_x_in_arena = 10.0
        self.car_pos_y_in_arena = 5.0
        self.car_orientation_in_arena = 0.0

        # car dimenstions
        # TODO: get the values for the car from the config or udp_manager
        self.car_length = 4.95  # in meters
        self.car_width = 1.96   # in meters

        # safe set
        self.safe_set_center_x = self.car_pos_x_in_arena
        self.safe_set_center_y = self.car_pos_y_in_arena
        self.safe_set_width = self.x_eta[0]
        self.safe_set_hight = self.x_eta[1]
        

        # start the visualizer in a new thread
        self.we_are_closing = False
        threading.Thread(target=self.visualizer_thread, daemon=True).start()

        # obstables and their update lock
        self.dynamic_object_list = []
        self.dynamic_object_list_lock = threading.Lock()


    def __del__(self):
        self.we_are_closing = True

    # safe set provided as lower-bound + dimensions
    def get_safe_set(self):
        safe_set_center_with_dimensions = [self.safe_set_center_x, self.safe_set_center_y, self.safe_set_width, self.safe_set_hight]
        safe_Set_lb_with_dimensions = [
            safe_set_center_with_dimensions[0] - safe_set_center_with_dimensions[2]/2.0,
            safe_set_center_with_dimensions[1] - safe_set_center_with_dimensions[3]/2.0,
            safe_set_center_with_dimensions[2],
            safe_set_center_with_dimensions[3]
        ]
        return safe_Set_lb_with_dimensions
    
    # target set provided as lower-bound + dimensions
    def get_target_set(self):
        target_set_width = 4*self.x_eta[0]
        target_set_center_with_dimensions = [
            self.safe_set_center_x + self.safe_set_width/2.0 - target_set_width/2.0,
            self.safe_set_center_y,
            target_set_width,
            self.safe_set_hight
        ]
        target_set_lb_with_dimensions = [
            target_set_center_with_dimensions[0] - target_set_center_with_dimensions[2]/2,
            target_set_center_with_dimensions[1] - target_set_center_with_dimensions[3]/2,  
            target_set_center_with_dimensions[2],
            target_set_center_with_dimensions[3]
        ]
        return target_set_lb_with_dimensions

    # all control sets provided as lower-bound + dimensions
    def get_control_sets(self):
        dynamic_objects = []
        with self.dynamic_object_list_lock:
            dynamic_objects=  [
                [obj.position.x - obj.length/2.0,
                 obj.position.y - obj.width/2.0,
                 obj.length, 
                 obj.width]  for obj in self.dynamic_object_list]
        
        return [dynamic_objects, self.get_safe_set(), self.get_target_set()]


    def visualizer_thread(self):
        self.arena_visualizer = ArenaVisualizer([self.car_pos_x_in_arena, self.car_pos_y_in_arena, self.car_orientation_in_arena], self.config_reader, self.get_control_sets)
        self.arena_visualizer.start()
        
        # sleep till closing signal is received
        while not self.we_are_closing:
            time.sleep(0.1)


    # Returns the point relative target frame (Ego's Frame) given a point in the standard frame (Simulator's World Frame)
    #
    # We know that the following holds from the frame transformation definition:
    #
    # " point_in_standard_frame = R * point_in_target_frame + target_frame_base "
    #
    # where, R is the rotation matrix based on theta = target_frame_angle
    # Therefore, we can rearrange the equation to find point_in_target_frame as follows:
    #
    # " point_in_target_frame = R^T * ( point_in_standard_frame - target_frame_base ) "
    #
    # where R^T is the transpose of R (which is also its inverse since R is orthogonal)
    #
    def getPointInTargetFrame2D(self, target_frame_base, target_frame_angle, point_in_standard_frame):
        standard_minus_base_x = point_in_standard_frame[0] - target_frame_base[0]
        standard_minus_base_y = point_in_standard_frame[1] - target_frame_base[1]
        point_in_target_frame_x = (+math.cos(target_frame_angle)*standard_minus_base_x) + (+math.sin(target_frame_angle)*standard_minus_base_y)
        point_in_target_frame_y = (-math.sin(target_frame_angle)*standard_minus_base_x) + (+math.cos(target_frame_angle)*standard_minus_base_y)
        return [point_in_target_frame_x, point_in_target_frame_y]
    

    # Decides which pFaces-SymControl action to promote as the final control input to be sent to the vehicle
    def promote_best_action(self, pfaces_sym_control_actions, unschielded_control_input):
        
        best_action = unschielded_control_input
        min_distance = float('inf')
        
        for action in pfaces_sym_control_actions:
            distance = linalg.norm([
                action[0] - unschielded_control_input.accel, 
                action[1] - unschielded_control_input.steering
            ])
            if distance < min_distance:
                min_distance = distance
                best_action = action
    
        return best_action
        

    def check_safety(self, vehicle_state, object_list, unschielded_control_input):

        # We define a target frame for the vehicle's moving arena that will be provided to pFaces. The base point of the frame is
        # the vehicle's position. The orinetation of the frame is the yaw of the vehicle.
        # - Target frame:
        #   base: vehicle's position = [vehicle_state.position.x, vehicle_state.position.y]
        #   angle: vehicle's yaw = vehicle_state.yaw
        target_frame_base = [vehicle_state.position.x, vehicle_state.position.y]
        target_frame_angle = vehicle_state.yaw

        # Build scene for pFaces-SymControl from vehucle_state and dynamic_object_list
        # We need: (1) obstacles from objects, (2) safe/target set based on the specs .. we will see.
        with self.dynamic_object_list_lock:

            # get a new copy of objects dedicated to us as we will edit it
            self.dynamic_object_list = copy.deepcopy(object_list)

            # adjust positions to be relative to arena-base (not vehicle)
            for obj in self.dynamic_object_list:
                obj_relative_pos = self.getPointInTargetFrame2D(target_frame_base, target_frame_angle, [obj.position.x, obj.position.y])
                obj.position = Point(
                    obj_relative_pos[0] + self.car_pos_x_in_arena, 
                    obj_relative_pos[1] + self.car_pos_y_in_arena
                )

            # inflate objects with the size of our vehicle as safety measure
            # TODO: for now, lenth is added to lengths and width is added to widths, but we should consider the vehicle orientation
            for obj in self.dynamic_object_list:
                obj.length += self.car_length
                obj.width += self.car_width

            # remove any objects that are outside of the arena
            self.dynamic_object_list = [obj for obj in self.dynamic_object_list if 
                                        ((obj.position.x + obj.length/2.0) >= 0.0 and (obj.position.x - obj.length/2.0) <= self.arena_width and 
                                            (obj.position.y + obj.width/2.0) >= 0.0 and (obj.position.y - obj.width/2.0) <= self.arena_height)]
            
        # Find a safe set by expnding the ego vehicle rectangle until it can no longer be expanded due to the obstacles (the objects).
        can_expand_x = True
        can_expand_y = True
        self.safe_set_center_x = self.car_pos_x_in_arena
        self.safe_set_center_y = self.car_pos_y_in_arena
        self.safe_set_width = self.x_eta[0]
        self.safe_set_hight = self.x_eta[1]
        new_length = self.safe_set_width
        new_width = self.safe_set_hight
        while can_expand_x or can_expand_y:

            if can_expand_x:
                new_length = self.safe_set_width + 2*self.x_eta[0]
                new_width = self.safe_set_hight
                new_center_x = self.safe_set_center_x + self.x_eta[0] # to keep the car at the left side of the safe set
                expanding_direction = 'x'
            else:
                new_width = self.safe_set_hight + 2*self.x_eta[1]
                new_length = self.safe_set_width
                new_center_x = self.safe_set_center_x
                expanding_direction = 'y'

            half_length = new_length / 2.0
            half_width = new_width / 2.0

            collision = False
            for obj in self.dynamic_object_list:
                # AABB collision check
                if (abs(new_center_x - obj.position.x) <= (half_length + obj.length/2.0)) and \
                    (abs(self.safe_set_center_y - obj.position.y) <= (half_width + obj.width/2.0)):
                    collision = True
                    
                    if expanding_direction == 'x':
                        can_expand_x = False
                    else:
                        can_expand_y = False

                    break
            
            if not collision:
                self.safe_set_width = new_length
                self.safe_set_hight = new_width  
                self.safe_set_center_x = new_center_x

            # check arena bounds
            if (self.safe_set_center_x - half_length) < 0.0 or (self.safe_set_center_x + half_length) > self.arena_width:
                can_expand_x = False
            if (self.safe_set_center_y - half_width) < 0.0 or (self.safe_set_center_y + half_width) > self.arena_height:
                can_expand_y = False
            
        # Collect and prepare obstacles and target sets for pFaces-SymControl
        [obstacles, _, target_set] = self.get_control_sets()
        obstacles_as_intervals = [[obstable[0], obstable[0] + obstable[2], obstable[1],  obstable[1] + obstable[3], -3.14, 3.14] for obstable in obstacles]
        obstacles_data = [list2str(obst_interval) for obst_interval in obstacles_as_intervals]
        target_set_as_interval = [target_set[0], target_set[0] + target_set[2], target_set[1], target_set[1] + target_set[3], -3.14, 3.14]
        target_date = list2str(target_set_as_interval)
        
        # Call pFaces-SymControl client and send the synthesis request:
        is_last_synth_request = False
        self.client.send_synthesis_request(obstacles_data, target_date, is_last_synth_request)

        # Call pFaces-SymControl client and get the control actions.
        # From pFaces prespective, the world is changin around the vehicle, therefore the current state is always fixed.
        current_state = [
            self.car_pos_x_in_arena,        # x is fixed in its frame
            self.car_pos_y_in_arena,        # y is fixed in its frame
            0.0                             # angle is fixed in its frame
        ]
        is_last_control_request = True
        actions = self.client.get_control_actions(current_state, is_last_control_request)
        
        # Pick best action to promote
        return self.promote_best_action(actions, unschielded_control_input)
