#!/usr/bin/env python
import rospy
import cv2
import copy
import numpy as np
import numpy.ma as ma
import math
import sys
from core_msgs.msg import PathArray
from core_msgs.msg import Control
from core_msgs.msg import CenPoint
from core_msgs.msg import VehicleState
from core_msgs.msg import Estop
from core_msgs.msg import ParkPoints
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



Z_DEBUG = False

class Control_mode(object):
    NO_CONTROL = 0
    EMERGENCY_BRAKE = 1
    NORMAL_S = 2 # normal s path following mode
    NORMAL_C = 3 # normal c point following mode
    SLOW_C = 4 # c poin following with low speed
    U_TURN = 5 # when u turn is activated
    PARK_FORWARD = 6
    PARK_REVERSE = 7 # when returning to main lane from parking lot

class tracker :
      def __init__ (self) :
            # -------------^CONSTANT^--------------
            self.pub_rate = 20.0 #Hz   #IMPORTANT!!! Main control rate
            self.access_wait_rate = 500 #Hz
            self.speed_max = 2.2 #m/s
            # self.speed_max = 2.0
            self.speed_min = 0.45

            self.speed_slope = self.speed_max / 10 #m/s/conf
            self.path_error_max = 0.3 #m maximum error ############################# added : name 
            self.speed_uturn = 0.4
            self.speed_delta = 0.1
            self.p_gain = 0.3
            self.steer_p_final = 1.5
            self.steer_p_final_max = 1.8
            self.emergency_stop_keep = 3 #sec  #time keeping for emergency stop after receiving the message
            self.map_resolution = 0.03
            self.map_height = 200
            self.map_width = 200
            self.brake_max = 200 #this is for emergency braking(maximum braking)
            self.brake_adjust_max = 50 #this is for normal velocity adjustment
            self.brake_slope = 50
            self.lpf_dt_cutoff = 10

            self.steer_max = 28.1 #degree, this is for uturning ################### added
            # self.uturn_end = 100
            self.uturn_end = 280 #degree, turn 180 degree in uturning ####################### added
            self.goal_reach_distance = 0.5

            #JUNHO
            self.spath_length_threshold = 20 #TODO Tune this

            self.spath_time_threshold = 15 #sec
            # if Z_DEBUG:
            #       self.spath_time_threshold =1

            # ----------------------------------^HARDWARE_CONSTANT^------------------------------
            self.L = 1.02 #m distance between two wheel axis
            self.lr = 0.51 #m  distance between cm and rear wheel
            # JUNHO ??
            self.cencorr = -0.04 #m  origin distance between cenPoint and sPath
            #car coordinate origin will be the geometrical center of car frame, except bumper.
            #goalpoint should be cenPoint.y_waypoint + self.cencorr



            self.control = Control() #my order to the car
            self.control_mode = Control_mode.NO_CONTROL
            self.goalpoint = CenPoint() #data of goal point - sPath origin placed in center, cenPath origin placed 770 from front wheel

            self.sPath = PathArray()
            self.cPoint = CenPoint()
            self.pPoint = ParkPoints() #for parking mission
            self.updated_sPath = PathArray()
            self.updated_cPoint = CenPoint()
            self.updated_pPoint = ParkPoints() #for parking mission
            self.updated_pPoint.goal_point.x = -1.0 #needed for safe initialization
            self.rPath = PathArray()
            self.updated_rPath = PathArray()
            self.uturn_angle = 0 ################################ added

            self.estoptime = -10
            self.accessing_state = 0
            self.accessing_sPath = 0
            self.writing_state = 0
            #-----------------------^DETERMINED^---------------------
            self.flag_obstacle = 0

            self.current_state = VehicleState()
            self.state_buff = []
            self.state_time_buff = np.empty(0)
            self.state_buff_park = []
            self.state_time_buff_park = np.empty(0)

            self.control_buff = []
            self.control_time_buff = np.empty(0)
            self.state_buff_size = 600
            self.control_buff_size = 600 #buff_sizes are only used when updating the buffer
            self.control_count = 0
            # self.park_stop_count = 12*self.pub_rate
            # self.park_reverse_count = 13*self.pub_rate
            # self.uturn_stop_count = 4*self.pub_rate
            # self.park_semi_stop_count = 4*self.pub_rate
            self.park_stop_duration = 12
            self.park_reverse_duration = 14
            self.uturn_stop_duration = 2
            self.park_semi_stop_duration = 2

            self.park_stop_start_time= 0
            self.park_reverse_start_time =0
            self.uturn_stop_start_time= 0
            self.park_semi_stop_start_time= 0

            self.park_stop_toggle = True
            self.park_reverse_toggle = True
            self.uturn_stop_toggle = True
            self.park_semi_stop_toggle = True

            self.sPath_n = 10

            self.occupancy_map = np.zeros((self.map_height,self.map_width), dtype='uint8')
            #-------^INPUT^---------------

      def decide_sPath_n(self) : ########################################################## added
            num = 0
            err = 0
            point_x = []
            point_y = []
            path_err = []
            
            #change coordinate from occupancy map to car_cm_coordinate
            for i in self.updated_sPath.pathpoints :
                  point_x.append(self.map_resolution*(self.map_height-i.y))
                  point_y.append(self.map_resolution*(self.map_width/2-i.x))
                  num += 1

            #calculate error of each path
            for j in range(num) :
                  path_err.append(0.0)
                  if point_x[j] == 0:
                        if point_y[j] > 0 :
                              alpha = np.pi/2
                        elif point_y[j] == 0 :
                              alpha = 0
                        else :
                              alpha = -np.pi/2
                  else :
                        alpha = np.arctan(point_y[j]/point_x[j])

                  if alpha != 0 :
                        ld = np.sqrt(point_x[j] * point_x[j] + point_y[j] * point_y[j])
                        cen_x = -self.lr
                        cen_y = (ld + 2 * self.lr * np.cos(alpha)) / (2 * np.sin(alpha))
                        R = np.sqrt(cen_x * cen_x + cen_y * cen_y)
                        for k in range(j) :
                              dist = np.sqrt((point_x[k] - cen_x) * (point_x[k] - cen_x) + (point_y[k] - cen_y)* (point_y[k] - cen_y))
                              tmp = np.absolute(dist - R)
                              if path_err[j] < tmp :
                                    path_err[j] = tmp
                  else :
                        for k in range(j) :
                              tmp = np.absolute(point_y[k])
                              if path_err[j] < tmp :
                                    path_err[j] = tmp

            #find largest n with error under the maximum error
            for j in range(num) :
                  if path_err[j] < self.path_error_max :
                        self.sPath_n = j
                        err = path_err[j]
            
            return err


      #get the most current state in the state_buff for the given _time, the returned state's timestamp is always "before" the given _time
      def get_current_state(self, _time) :
            while self.writing_state >= 1 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.accessing_state += 1

            #make sure that state_buffer does not get updated whill getting the current state, and make sure the temporary buffers' sizes are equal to each other.
            temp_state_buff = copy.copy(self.state_buff)
            temp_state_time_buff = copy.copy(self.state_time_buff)
            if len(temp_state_buff) >= len(temp_state_time_buff):
                  temp_state_time_buff = temp_state_time_buff[:len(temp_state_buff)]
            else:
                  temp_state_buff = temp_state_buff[:len(temp_state_time_buff)]

            before_state_buff = []
            #TODO: make sure that state_buff and state_time_buff size is same
            is_before_buff = temp_state_time_buff < _time
            is_before_size = 0
            for i in range(len(is_before_buff)):
                  if not is_before_buff[i]:
                        is_before_size = i
                        break

            if is_before_size == 0:
                  return None, None, None

            before_state_buff= temp_state_buff[:is_before_size]
            before_state_time_buff = temp_state_time_buff[:is_before_size]
            after_state_time_buff = temp_state_time_buff[temp_state_time_buff > _time]

            _state = before_state_buff[-1]
            _state_time = before_state_time_buff[-1]
            if len(after_state_time_buff) is not 0:
                  _next_time = after_state_time_buff[0]
            else:
                  _next_time = None
            self.accessing_state -= 1
            return _state, _state_time, _next_time

      #IMPORTANT: is_transform_success: False when transform fails due to short state buffer
      def transform_pathpoint(self, vector_point, _past_t, _current_t):
            self.accessing_state += 1
            updated_vector_point = vector_point
            time_indicator = _past_t
            is_exit = False
            _first_time_indicator = 0
            # print "one transform starts"

            #use epsilon
            while time_indicator <= _current_t and not is_exit:
                  if time_indicator == _current_t:
                        # print "is_exit gets True"
                        is_exit = True
                  using_state, using_state_time, next_time = self.get_current_state(time_indicator)
                  if using_state == None:
                        # print "while loop using_state is None"
                        break
                  if time_indicator == _past_t:
                        time_indicator = next_time
                        _first_time_indicator = next_time
                        continue
                  if time_indicator == _first_time_indicator:
                        delta_t = time_indicator - _past_t
                  else:
                        delta_t = time_indicator - using_state_time
                  beta = np.arctan( (self.lr / self.L) * np.tan( -using_state.steer / 180 * np.pi)) #atan2
                  if beta != 0 :
                        R = self.lr / np.sin(beta)
                        yaw_rate = using_state.speed / R
                        delta_phi = delta_t * yaw_rate
                        alpha = delta_phi/2+beta
                        ld = np.absolute(2*R*np.sin(alpha - beta))
                        dx = - ld * np.cos(alpha)
                        dy = ld * np.sin(alpha)
                        x_prime = updated_vector_point.x +dx
                        y_prime = updated_vector_point.y + dy
                        # print "dx: ", str(round(dx,3)), ", dy: ", str(round(dy,3)), ", ld: ", str(round(ld,3)), ", delta_phi: ", str(round(delta_phi,3))
                        updated_vector_point.x = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                        updated_vector_point.y = -np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime

                  else :
                        updated_vector_point.x = updated_vector_point.x - using_state.speed * delta_t
                  # print "updated_x: ", str(round(updated_vector_point.x,3)), ", updated_y: ", str(round(updated_vector_point.y,3))
                  #time indicator update
                  time_indicator = next_time
            # print "while loop terminated"
            # if is_exit:
            # print "one transform ends"
            self.accessing_state -= 1
            return updated_vector_point, is_exit


      def update_con_pid(self) : # final decision for control outputs
            if self.control_mode == Control_mode.EMERGENCY_BRAKE:
                  return
            if self.current_state == None:
                  return
            curspd = self.current_state.speed
            # curstr = self.current_state.steer
            # objspd = self.control.speed
            # spderr = objspd - curspd

            # self.control.speed = objspd + spderr * self.p_gain #adding p gain
            if self.control.speed < 0 :
                  self.control.speed = 0
            elif self.control.speed > 0.3 and curspd < 0.3:
                  self.control.speed = 1.0
            if self.control.speed > self.speed_max:
                  self.control.speed = self.speed_max
            
            if self.flag_obstacle < 40:
                  self.control.steer = self.control.steer * self.steer_p_final
            
            else:
                  self.control.steer = self.control.steer * self.steer_p_final_max
                  if self.control.steer is not 0:
                        steer_sign = self.control.steer / abs(self.control.steer)
                  else:
                        steer_sign = 1
                  curve_param = 0.56
                  self.control.steer = steer_sign * (28**(1.0-curve_param)) * abs(self.control.steer)**curve_param
                  if self.control.steer > 3.0:
                        self.control.steer = max(6.0, self.control.steer)
                  elif self.control.steer < -3.0:
                        self.control.steer = min(self.control.steer, -6.0)

            if self.control.steer > self.steer_max:
                  self.control.steer = self.steer_max
            elif self.control.steer < -self.steer_max:
                  self.control.steer = -self.steer_max
            
            speed_err = self.control.speed - self.current_state.speed
            if self.control.brake >= self.brake_max:
                  self.control.brake = self.brake_max
            elif speed_err < -0.2: #TODO consider braking condition
                  self.control.brake = min(self.brake_adjust_max, -speed_err * self.brake_slope)
            elif self.control.brake < 0:
                  self.control.brake = 0


      def update_lpf (self) :
            alpha = 1.0/self.pub_rate / (1.0/self.pub_rate + 1.0/self.lpf_dt_cutoff)

            if len(self.control_buff) > 0 :
                  self.control.speed = (1-alpha) * self.control_buff[-1].speed + alpha * self.control.speed
                  self.control.steer = (1-alpha) * self.control_buff[-1].steer + alpha * self.control.steer

      # to get rid of spath points in the occupied region
      def is_pathpoint_occupied(self, x, y) :
            # x, y in vehicle meter coordinate
            pix_x = int(self.map_width / 2 - y/self.map_resolution)
            pix_y = int(self.map_height -1 - x/self.map_resolution)
            if pix_x > self.map_width-1 or pix_x < 0:
                  # print "out of region"
                  return True
            elif pix_y > self.map_height - 1 or pix_y < 0:
                  # print "out of region"
                  return True
            elif self.occupancy_map[pix_y, pix_x] ==255:
                  # print "occupied"
                  return True
            else:
                  return False

      def update_spath(self, _current_t):
            self.accessing_sPath += 1
            is_no_red_before = False

            if len(self.sPath.pathpoints) == 0:
                  # print "length of sPath is zero"
                  return False
            temp_updated_sPath = PathArray()
            temp_updated_sPath.header.stamp = self.sPath.header.stamp
            _past_t = self.sPath.header.stamp.to_sec()
            if _past_t > _current_t:
                  self.updated_sPath = copy.deepcopy(self.sPath) 
                  return True
            i_count = 0
            for i in self.sPath.pathpoints :
                  pathpoint_i = Vector3()
                  pathpoint_i.x = self.map_resolution*(self.map_height-i.y) #0.03m per pixel
                  pathpoint_i.y = self.map_resolution*(self.map_width/2-i.x)
                  pathpoint_i, is_transform_success = self.transform_pathpoint(pathpoint_i, _past_t, _current_t)
                  
                  if not is_transform_success:
                        # print "transform failed in spath updating"
                        return False

                  v = Vector3()
                  v.z = 0
                  v.y = self.map_height - pathpoint_i.x / self.map_resolution
                  v.x = self.map_width/2 - pathpoint_i.y / self.map_resolution

                  if i_count == 0:
                        # print v.x, ", ", v.y
                        if not self.is_pathpoint_occupied(pathpoint_i.x, pathpoint_i.y):
                              temp_updated_sPath.pathpoints.append(v)
                  if is_no_red_before:
                        if v.y < self.map_height and v.y >= 0:
                              temp_updated_sPath.pathpoints.append(v)

                  if not self.is_pathpoint_occupied(pathpoint_i.x, pathpoint_i.y):
                        is_no_red_before = True
                  i_count +=1

            self.updated_sPath = temp_updated_sPath
            self.accessing_sPath -= 1
            return True

      def update_rpath(self, _current_t):
            return

      #TODO: we have to check if cpoint updates well
      def update_cpoint(self, _current_t):
            _past_t = self.cPoint.header.stamp.to_sec()
            # print "delta t during update",_current_t - _past_t
            if _past_t > _current_t:
                  self.updated_cPoint = copy.deepcopy(self.cPoint)
                  return True

            _goalpoint = Vector3()
            _goalpoint.x = self.cPoint.x_waypoint - self.cencorr
            _goalpoint.y = self.cPoint.y_waypoint
            # print "before_goal_x: ", str(round(_goalpoint.x,3)), ", before_goal_y: ", str(round(_goalpoint.y,3))
            _goalpoint, is_transform_success = self.transform_pathpoint(_goalpoint, _past_t, _current_t)

            self.updated_cPoint.header.stamp = self.cPoint.header.stamp
            self.updated_cPoint.confidence = self.cPoint.confidence
            
            # self.show_cPoint()

            self.updated_cPoint.x_waypoint = _goalpoint.x
            self.updated_cPoint.y_waypoint = _goalpoint.y
            #HERE!!!
            # print "one update cpoint finish"
            # self.show_cPoint()
            
            if not is_transform_success:
                return False
            return True

      def update_ppoint(self, _current_t):
            _past_t = self.pPoint.header.stamp.to_sec()
            if len(self.pPoint.initpoints) == 0:
                  # print "pPoint not received"
                  return False
            if _past_t > _current_t:
                  self.updated_pPoint = copy.deepcopy(self.pPoint)
                  # print "past t > current t"
                  return True

            _goalpoint = Vector3()
            _goalpoint.x = self.pPoint.goal_point.x
            _goalpoint.y = self.pPoint.goal_point.y
            _goalpoint, is_transform_success = self.transform_pathpoint(_goalpoint, _past_t, _current_t)

            if not is_transform_success:
                  # print "first fail ppoint"
                  return False

            _init1point = Vector3()
            _init1point.x = self.pPoint.initpoints[0].x
            _init1point.y = self.pPoint.initpoints[0].y
            _init1point, is_transform_success = self.transform_pathpoint(_init1point, _past_t, _current_t)

            if not is_transform_success:
                  # print "second fail ppoint"
                  return False

            _init2point = Vector3()
            _init2point.x = self.pPoint.initpoints[1].x
            _init2point.y = self.pPoint.initpoints[1].y
            _init2point, is_transform_success = self.transform_pathpoint(_init2point, _past_t, _current_t)

            if not is_transform_success:
                  # print "third fail ppoint"
                  return False

            self.updated_pPoint.header.stamp = self.pPoint.header.stamp
            self.updated_pPoint.goal_point = _goalpoint
            self.updated_pPoint.initpoints = [_init1point, _init2point]
            return True

      def distance_to_point(self, _point):#_point is in CenPoint type
            distance = math.sqrt(_point.x_waypoint**2 + _point.y_waypoint**2)
            # print "distance to goalpoint is ", distance
            return distance

      def make_reverse_path(self):
            if len(self.state_buff_park) > len(self.state_time_buff_park):
                  self.state_buff_park = self.state_buff_park[:len(self.state_time_buff_park)]
            elif len(self.state_buff_park) < len(self.state_time_buff_park):
                  self.state_time_buff_park = self.state_time_buff_park[:len(self.state_buff_park)]
            state_length = len(self.state_buff_park)
            _path_point = Vector3()
            _path_point.x = 0
            _path_point.y = 0
            prev_time = self.state_time_buff_park[-1]
            i = 0
            for r_state in reversed(self.state_buff_park):
                  if i == 0:
                        continue
                  curr_time = self. state_time_buff_park[state_length - i - 1]
                  delta_t = prev_time - curr_time
                  beta = np.arctan(self.lr / self.L * np.tan( -r_state.steer / 180 * np.pi))
                  if beta != 0 :
                        R = self.lr / np.sin(beta)
                        yaw_rate = r_state.speed / R
                        delta_phi = delta_t * yaw_rate
                        alpha = delta_phi/2+beta
                        ld = np.absolute(2*R*np.sin(alpha - beta))
                        x_prime = _path_point.x + ld * np.cos(alpha)
                        y_prime = _path_point.y - ld * np.sin(alpha)
                        _path_point.x = np.cos(delta_phi) * x_prime - np.sin(delta_phi) * y_prime
                        _path_point.y = np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime

                  else :
                        _path_point.x = _path_point.x + r_state.speed * delta_t

                  self.rPath.pathpoints.append(_path_point)
                  prev_time = curr_time
                  i += 1
            return

      def main_control_loop(self) :

            '''
            1. Initialization
            '''
            self.control = Control()
            temp_control_mode = Control_mode.NO_CONTROL
            self.control.is_auto = True
            self.control.estop = 0
            self.control.gear = 0 #0: front 1: neutral 2: rear
            self.control.speed = 0
            self.control.steer = 0 #in degree
            self.control.brake = 1
            #TODO: change this to server client later
            #uturn = 0
            uturn_mode = rospy.get_param('uturn_mode') ###################### added
            park_mode = rospy.get_param('park_mode')
            current_t = self.current_state.header.stamp.to_sec()

            '''
            2. Updating the s-path and c-point according to vehicle odometry
            '''   

            #update s-path
            #erase s-path to empty object after a certain period of time
            #JUNHO: this does not work well in rosbag mdode because current_t is much more current than the stamp
            #IMPORTANT: emergency_brake case have move to the top for fast response
            if current_t - self.estoptime < self.emergency_stop_keep :
                  if Z_DEBUG:
                        print "control mode: emergency braking"
                  temp_control_mode = Control_mode.EMERGENCY_BRAKE

            if temp_control_mode is not Control_mode.EMERGENCY_BRAKE:            
                  if current_t - self.sPath.header.stamp.to_sec() > self.spath_time_threshold:
                        self.sPath.pathpoints = []
                        self.updated_sPath.pathpoints = []
                        # print "path erased"

                  is_updated_spath= self.update_spath(current_t)
                  # if Z_DEBUG and not is_updated_spath :
                        # print "s-path is not updated and is empty"
                  #update c-point
                  is_updated_cpoint = self.update_cpoint(current_t)
                  # if Z_DEBUG and not is_updated_cpoint :
                        # print "c-point is not updated due to short state buffer"
                  if park_mode ==2:
                        is_updated_ppoint = self.update_ppoint(current_t)
                  
                  elif park_mode ==3:
                        is_updated_rpath = self.update_rpath(current_t)

            
            '''
            3. Deciding the control mode
            IMPORTANT: This is the most upper level logic of low-level controller!!
            '''
            # emergency stop case, emergency stop occurs by the e_stop message
            if temp_control_mode == Control_mode.EMERGENCY_BRAKE:
                  if Z_DEBUG:
                        print "control mode: emergency braking"
            elif uturn_mode >= 1: ##################################### added
                  if uturn_mode == 1:
                        if self.uturn_stop_toggle:
                              self.uturn_stop_start_time = rospy.Time.now().to_sec()
                              self.uturn_stop_toggle = True
                        if Z_DEBUG:
                              print "control mode: u-turning enter, C point tracking with low speed"
                        temp_control_mode = Control_mode.SLOW_C
                  elif uturn_mode == 2:
                        ##TODO: upgrade second logic of if statement
                        # if self.uturn_angle > self.uturn_end and self.cpoint.header.stamp.to_sec() > self.control_time_buff[-1]:      
                        if self.uturn_angle > self.uturn_end:      
                              if Z_DEBUG:
                                    print "control mode: u-turning end, C point tracking start"
                              uturn_mode = 0
                              rospy.set_param("uturn_mode", 0)
                              # self.uturn_angle = 0
                              temp_control_mode = Control_mode.NORMAL_C
                        else :
                              # if Z_DEBUG:
                                    # print "control mode: u-turning"
                              temp_control_mode = Control_mode.U_TURN
            elif park_mode >= 1:
                  if park_mode ==1:
                        temp_control_mode = Control_mode.SLOW_C
                        if self.park_semi_stop_toggle:
                              self.park_semi_stop_start_time = rospy.Time.now().to_sec()
                              self.park_semi_stop_toggle = False
                  elif park_mode ==2:
                        temp_control_mode = Control_mode.PARK_FORWARD
                  elif park_mode ==3:
                        temp_control_mode = Control_mode.PARK_REVERSE
                        if self.park_stop_toggle:
                              self.park_stop_start_time = rospy.Time.now().to_sec()
                              self.park_stop_toggle = False
                        
            # normal s path tracking, when the updated_sPath is still long enough to follow, the vehicle follows it.
            # This situation occurs during inside missions with rubber cones, and also when the vehicle is exiting the mission.
            elif len(self.updated_sPath.pathpoints) > self.spath_length_threshold :
                  if Z_DEBUG:
                        print "control mode: Normal S path tracking"
                  temp_control_mode = Control_mode.NORMAL_S

            # c point tracking with low speed
            # This situation occurs when sPath is not created but there are obstacles
            # Usually occurs when entering into the mission with rubber cones, or when the sPath keeps fail updating during the mission.
            elif self.flag_obstacle > 0:
                  if Z_DEBUG:
                        print "control mode: C point tracking with low speed, there still are obstacles."
                  temp_control_mode = Control_mode.SLOW_C

            else: #when flag_obstacle == 0
                  # if Z_DEBUG:
                        # print "control mode: Normal C point tracking"
                  temp_control_mode = Control_mode.NORMAL_C
            #TODO add cases for uturn and parking control
            #TODO add case to deal with when cPoint is also empty
            self.control_mode = temp_control_mode
            '''
            4. Decide steering angle depending on each control mode
                  1) updtate goalpoint
                  if control mode is NORMAL_S, decide goalpoint according to sPath_n and updated_sPath
                  if control mode is NORMAL_C or SLOW_C, decide goalpoint according and updated_cPoint
                  2) pure pursuit logic
            '''
            # 1) update goalpoint
            # self.show_cPoint()
            if self.control_mode == Control_mode.NORMAL_S:
                  #TODO: activate this
                  # self.decide_sPath_n() ########################################### added
 
                  if len(self.updated_sPath.pathpoints) <= self.sPath_n :
                        self.goalpoint.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[0].y)
                        self.goalpoint.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[0].x)
                  else:
                        self.goalpoint.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[-self.sPath_n].y)
                        self.goalpoint.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[-self.sPath_n].x)

            elif self.control_mode == Control_mode.NORMAL_C or self.control_mode == Control_mode.SLOW_C:
                  self.goalpoint.x_waypoint = self.updated_cPoint.x_waypoint
                  self.goalpoint.y_waypoint = self.updated_cPoint.y_waypoint
                  self.goalpoint.confidence = self.updated_cPoint.confidence
            elif self.control_mode == Control_mode.PARK_FORWARD:
                  self.goalpoint.x_waypoint = self.updated_pPoint.goal_point.x
                  self.goalpoint.y_waypoint = self.updated_pPoint.goal_point.y
                  # print self.goalpoint
                  if self.goalpoint.x_waypoint < -0.2:
                        print "goal point passed"
                  if self.updated_pPoint.goal_point.x >= -1.0 and ( self.distance_to_point(self.goalpoint) < self.goal_reach_distance or (self.goalpoint.x_waypoint < -0.2 and abs(self.goalpoint.y_waypoint) < 0.5)) :
                        rospy.set_param('park_mode',3)
                        # self.make_reverse_path()
            elif self.control_mode == Control_mode.PARK_REVERSE:
                  if current_t - self.park_reverse_start_time > self.park_reverse_duration and self.park_reverse_start_time is not 0:
                        print "park reverse count finish"
                        rospy.set_param('park_mode', 0)      
            # in other control_mode, goalpoint do not need to be set again because the control logic does not depend on the goalpoint.

            # 2) deciding PURE PURSUIT OUTPUT based on goalpoint
            if self.control_mode == Control_mode.EMERGENCY_BRAKE:
                  delta = 0
            elif self.control_mode == Control_mode.NORMAL_S or self.control_mode == Control_mode.NORMAL_C or self.control_mode == Control_mode.SLOW_C or self.control_mode == Control_mode.PARK_FORWARD:
                  ld = math.sqrt(self.goalpoint.x_waypoint*self.goalpoint.x_waypoint + self.goalpoint.y_waypoint*self.goalpoint.y_waypoint)
                  if ld < 0.2:
                        delta  = 0
                  else:
                        delta = -np.arctan(2 * self.L * (self.goalpoint.y_waypoint / ld) / (ld + 2 * self.lr * self.goalpoint.x_waypoint/ld)) # ld is lookahead distance
                  self.control.steer = delta / np.pi * 180 #in degree
                  # print "first steer", self.control.steer

            #################### added
                  
            elif self.control_mode == Control_mode.U_TURN :
                  self.control.steer = -self.steer_max
                   #################### added
            
            elif self.control_mode == Control_mode.PARK_REVERSE:
                  self.control.steer = self.steer_max/2
            else:
                  self.control.steer = 0.0
            #TODO: set steer for u-turn and reverse


            if self.control_mode == Control_mode.PARK_REVERSE:
                  self.control.gear = 2
            else:
                  self.control.gear = 0

            '''
            5. Decide speed depending on each control mode
            '''
            if self.control_mode == Control_mode.EMERGENCY_BRAKE:
                  self.control.speed = 0
                  self.control.brake = self.brake_max
            elif park_mode == 1:
                  if current_t - self.park_semi_stop_start_time <= self.park_semi_stop_duration:
                        self.control.speed = 0
                        self.control.brake = self.brake_max

                  elif self.park_semi_stop_start_time is not 0:
                        self.control.speed = self.speed_min

            elif self.control_mode == Control_mode.NORMAL_C:
                  self.control.speed = self.goalpoint.confidence * self.speed_slope
                  # print(self.goalpoint.confidence)
            elif self.control_mode == Control_mode.NO_CONTROL:
                  self.control.speed = 0
            #TODO: set speed for u-turn and reverse
            
            elif self.control_mode == Control_mode.U_TURN:
                  if current_t - self.uturn_stop_start_time <= self.uturn_stop_duration:
                        self.control.speed = 0
                        self.control.brake = self.brake_max
                  # elif self.uturn_stop_start_time is not 0:
                  else:
                        self.control.speed = self.speed_uturn

            elif self.control_mode == Control_mode.PARK_REVERSE:
                  if current_t - self.park_stop_start_time <= self.park_stop_duration:
                        self.control.speed = 0
                        self.control.brake = self.brake_max

                  elif self.park_stop_start_time is not 0:
                        if self.park_reverse_toggle:
                              self.park_reverse_start_time = rospy.Time.now().to_sec()
                              self.park_reverse_toggle = False
                        self.control.speed = self.speed_min
            else:
                  self.control.speed = self.speed_min


            '''
            6. Additional Post processing on Control commands
                  1) Add P gains to Steer and Speed command and Decide brake
                  2) Apply low pass filter
                  3) Updtate Control buffer
            '''
            #final decision making for control variables (adding p gains and decide brake value)
            self.update_con_pid()
            #applying low pass filter to finally decided control variable
            self.update_lpf()

            #updating control buffer
            # if Z_DEBUG:
            #       print "length of control time buffer is ", len(self.control_time_buff)
            if len(self.control_buff) >= self.control_buff_size:
                  self.control_buff[0:-1]=self.control_buff[1:]
                  self.control_buff[-1] = self.control
                  self.control_time_buff[0:-1] = self.control_time_buff[1:]
                  self.control_time_buff[-1] = rospy.Time.now().to_sec()
            else:
                  self.control_buff.append(self.control)
                  self.control_time_buff = np.append(self.control_time_buff, rospy.Time.now().to_sec())

            self.control.control_mode = self.control_mode

            self.control_count +=1
            rospy.set_param('control_count',self.control_count)

            return self.control #need to publish self.control as control message

            #return to access free mode
            #exit

      #function that write, show, get the data of class
      def write_em(self, data) :
            self.estoptime = data.header.stamp.to_sec()

      def write_occupancy(self, map_image):
            self.occupancy_map = map_image[:,:,2]

      def write_cPoint(self, data) :
            self.cPoint=data

      def write_pPoint(self, data) :
            self.pPoint = data

      def write_sPath(self, data) :
            #make sure that writing new sPath does not occurs during updating sPath
            # while self.accessing_sPath >= 1 :
            #       rospy.Rate(self.access_wait_rate).sleep()
            if len(data.pathpoints) > 0 :
                  self.sPath=data

      def write_stbuff(self, data) :
            self.writing_state = 1
            self.current_state = data
            if len(self.state_buff) >= self.state_buff_size:
                  self.state_buff[0:-1]=self.state_buff[1:]
                  self.state_buff[-1]=data
                  self.state_time_buff[0:-1] = self.state_time_buff[1:]
                  self.state_time_buff[-1] = self.current_state.header.stamp.to_sec()
            else:
                  self.state_buff.append(data)
                  self.state_time_buff = np.append(self.state_time_buff, self.current_state.header.stamp.to_sec())

            if self.control_mode == Control_mode.U_TURN : ################################### added
                  beta = np.arctan(self.lr / self.L * np.tan(-data.steer*np.pi/180))
                  if beta == 0:
                        return
                  R = self.lr / np.sin(beta)
                  # print "R is ",R
                  angle = 0.0
                  if len(self.state_time_buff) >=2:
                        angle = data.speed / R * (self.state_time_buff[-1] - self.state_time_buff[-2])
            
                  self.uturn_angle += (angle / np.pi) * 180 ####################### added

            if self.control_mode == Control_mode.PARK_FORWARD:
                  self.state_buff_park.append(data)
                  self.state_time_buff_park = np.append(self.state_time_buff, self.current_state.header.stamp.to_sec())                  

            self.writing_state = 0

      def show_sPath(self) :
            for i in self.sPath.pathpoints :
                  print i

      def show_cPoint(self) :
            print self.cPoint

      def show_stbuff(self) :
            print self.state_buff

      def show_control_buff(self) :
            for i in self.control_buff :
                  print i

      def show_control(self) :
            print self.control

      def get_sPath(self) :
            return self.sPath

      def get_cPoint(self) :
            return self.cPoint


      def get_control_buff(self, i) :
            return self.control_buff[i]

      def get_control(self) :
            return self.control

main_track = tracker()

def callback_s(data) :
      main_track.write_sPath(data)
#      main_track.show_sPath()
#      print "Got sPath"

def callback_c(data) :
      main_track.write_cPoint(data)
#      main_track.show_cPoint()
#      print "Got cPoint"

def callback_p(data) :
      print "new park point received"
      main_track.write_pPoint(data)

def callback_stbuff(data) :
      main_track.write_stbuff(data)
#      main_track.show_stbuff()
#      print "Got state_buff"

def callback_emergency(data) :
      if data.estop >= 1:
            main_track.write_em(data)

def callback_obstacle(data):
      main_track.flag_obstacle = data.data

def callback_occupancy(map):
      map_image = bridge.imgmsg_to_cv2(map, "bgr8")
      main_track.write_occupancy(map_image)

def init() :
      rospy.init_node('path_tracker', anonymous=True)
      rospy.Subscriber('/sPath', PathArray, callback_s)
      rospy.Subscriber('/waypoints', CenPoint, callback_c)
      rospy.Subscriber('/initial_points_for_park', ParkPoints, callback_p)
      rospy.Subscriber('/vehicle_state', VehicleState, callback_stbuff, queue_size = 1)
      rospy.Subscriber('/emergency_stop', Estop, callback_emergency, queue_size =1)
      rospy.Subscriber('/flag_obstacle', Int32, callback_obstacle)
      rospy.Subscriber('/occupancy_map', Image, callback_occupancy, queue_size=1, buff_size=2**24)

      cont_pub = rospy.Publisher('/control', Control, queue_size=10)
      mon_path_pub = rospy.Publisher('/path_tracking', PathArray, queue_size=10)
      updated_cpoint_pub = rospy.Publisher('/updated_cpoint', CenPoint, queue_size=10)
      updated_ppoint_pub = rospy.Publisher('/updated_ppoint', ParkPoints, queue_size=10)
      spath_n_pub = rospy.Publisher('/lookahead_n', Int32, queue_size=10)
      uturn_angle_pub = rospy.Publisher('/uturn_angle', Float32, queue_size =1)      

      rate = rospy.Rate(main_track.pub_rate)
      #TODO: set vehicle parameters from yaml file

      while not rospy.is_shutdown() :
            cont_pub.publish(main_track.main_control_loop())
            mon_path_pub.publish(main_track.updated_sPath)
            spath_n_pub.publish(main_track.sPath_n)
            uturn_angle_pub.publish(main_track.uturn_angle)
            # main_track.show_cPoint()
            updated_cpoint_pub.publish(main_track.updated_cPoint)
            updated_ppoint_pub.publish(main_track.updated_pPoint)
            rate.sleep()


if __name__ == '__main__':
      # if len(sys.argv) < 2:
      #       print("usage: path_tracker_new.py use_rosbag")
      # else:
      bridge = CvBridge()
      init()
