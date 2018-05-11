#!/usr/bin/env python
import rospy
import numpy as np
import numpy.ma as ma
import math
from core_msgs.msg import PathArray
from core_msgs.msg import Control
from core_msgs.msg import CenPoint
from core_msgs.msg import VehicleState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

Z_DEBUG = False

class Control_mode(object):
    NO_CONTROL = 0
    EMERGENCY_BRAKE = 1
    NORMAL_S = 2 # normal s path following mode
    NORMAL_C = 3 # normal c point following mode
    SLOW_C = 4 # c poin following with low speed
    U_TURN = 5 # when u turn is activated
    PARK_REVERSE = 6 # when returning to main lane from parking lot

class tracker :
      def __init__ (self) :
            # -------------^CONSTANT^--------------
            self.pub_rate = 20.0 #Hz   #IMPORTANT!!! Main control rate
            self.access_wait_rate = 500 #Hz
            self.speed_slope = 0.1 #m/s/conf
            self.delta_y_max = 0.3 #m maximum error
            self.speed_max = 2.1 #m/s
            self.speed_min = 0.5
            self.speed_delta = 0.1
            self.p_gain = 0.3
            self.emergency_stop_keep = 10 #sec  #time keeping for emergency stop after receiving the message
            self.map_resolution = 0.03
            self.map_height = 200
            self.map_width = 200
            self.brake_max = 200 #this is for emergency braking(maximum braking)
            self.brake_adjust_max = 50 #this is for normal velocity adjustment
            self.brake_slope = 50
            self.lpf_dt_cutoff = 10

            #JUNHO
            self.spath_length_threshold = 8

            self.spath_time_threshold = 30 #sec
            if Z_DEBUG:
                  self.spath_time_threshold =1

            # ----------------------------------^HARDWARE_CONSTANT^------------------------------
            self.L = 1.54 #m distance between two wheel axis
            self.lr = 0.53 #m  distance between cm and rear wheel
            # JUNHO ??
            self.cencorr = -0.04 #m  origin distance between cenPoint and sPath
            #car coordinate origin will be the geometrical center of car frame, except bumper.
            #goalpoint should be cenPoint.y_waypoint + self.cencorr



            self.control = Control() #my order to the car
            self.goalpoint = CenPoint() #data of goal point - sPath origin placed in center, cenPath origin placed 770 from front wheel

            self.sPath = PathArray()
            self.cPoint = CenPoint()
            self.updated_sPath = PathArray()
            self.updated_cPoint = CenPoint()

            #time variables _ time when this node get sPath or cenPoint
            # self.stime = 0 #JUNHO not using this, instead, using time stamp in spath
            #self.ctime = 0 #JUNHO not using this, instead, using time stamp in spath
            self.state_update_time = 0 #JUNHO: originally sttime
            self.estoptime = -10
            self.accessing_state = 0
            self.accessing_sPath = 0
            self.writing_state = 0
            #-----------------------^DETERMINED^---------------------
            self.flag_obstacle = 0

            self.state_buff = []
            self.state_time_buff = np.empty(0)
            self.control_buff = []
            self.control_time_buff = np.empty(0)
            self.state_buff_size = 100
            self.control_buff_size = 100 #buff_sizes are only used when updating the buffer

            self.sPath_n = 20
            #-------^INPUT^---------------

      def decide_sPath_n(self, vel) :
            number = 0
            cur_ind = CenPoint()
            deg = []
            p_ind = CenPoint()
            acc = 0.0
            for i in self.updated_sPath.pathpoints :
                  cur_ind.x_waypoint = self.map_resolution*(200-i.y) #0.3m per pixel
                  cur_ind.y_waypoint = self.map_resolution*(100-i.x)
                  deg.append((cur_ind.y_waypoint - p_ind.y_waypoint) / (cur_ind.x_waypoint - p_ind.x_waypoint))
                  number+=1
            for i in range(1,number) :
                  cur_ind.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[i].y)
                  cur_ind.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[i].x)
                  if cur_ind.x_waypoint == 0 :
                        alpha = np.arctan(cur_ind.y_waypoint / cur_ind.x_waypoint)
                  else :
                        alpha = np.pi/2
                  ld = math.sqrt(cur_ind.x_waypoint*cur_ind.x_waypoint + cur_ind.y_waypoint*cur_ind.y_waypoint)
                  beta = np.arctan(self.lr * np.cos(alpha) / (ld/2 + self.lr * np.cos(alpha)))
                  p_head = 2 * beta
                  head_change = 2 * (alpha - beta)
                  sum_error = 0
                  R = self.lr / np.sin(beta)
                  for j in range(i) :
                        if vel/self.pub_rate < R * p_head:
                              break
                        sum_error += deg[j] - p_head
                        if np.absolute(sum_error) >acc: #################################################
                              acc = np.absolute(sum_error)
                              self.n = i
                        p_head+=head_change/i
            return acc

      #get the most current state in the state_buff for the given _time, the returned state's timestamp is always "before" the given _time
      def get_current_state(self, _time) :
            while self.writing_state >= 1 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.accessing_state += 1

            before_state_buff = []
            #TODO: make sure that state_buff and state_time_buff size is same
            is_before_buff = self.state_time_buff < _time
            for i in range(len(is_before_buff)):
                  before_state_buff.append(self.state_buff[i])
            before_state_time_buff = self.state_time_buff[self.state_time_buff < _time]
            after_state_time_buff = self.state_time_buff[self.state_time_buff > _time]
            # if Z_DEBUG:
            #       print "lengths are ", len(before_state_buff), ", ",len(before_state_time_buff)
            if len(before_state_buff) == 0 or len(before_state_time_buff) == 0:
                  # if Z_DEBUG:
                  #       print "returning None"
                  return None, None, None

            _state = before_state_buff[-1]
            _state_time = before_state_time_buff[-1]
            if len(after_state_time_buff) is not 0:
                  _next_time = after_state_time_buff[0]
            else:
                  _next_time = None
            self.accessing_state -= 1
            return _state, _state_time, _next_time

      #JUNHO: transform_pathpoint is the new update_goalpoint
      #IMPORTANT: is_transform_success: False when transform fails due to short state buffer
      def transform_pathpoint(self, vector_point, _past_t, _current_t):
            self.accessing_state += 1
            updated_vector_point = vector_point
            time_indicator = _past_t
            is_exit = False
            is_success = False
            _first_time_indicator = 0
            while time_indicator <= _current_t and not is_exit:
                  if time_indicator == _current_t:
                        is_exit = True
                  using_state, using_state_time, next_time = self.get_current_state(time_indicator)
                  if using_state == None:
                        # print "while loop using_state is None"
                        break
                  if time_indicator == _past_t:
                        time_indicator = next_time
                        _first_time_indicator = time_indicator
                        continue
                  if time_indicator == _first_time_indicator:
                        delta_t = time_indicator - _past_t
                  else:
                        delta_t = time_indicator - using_state_time
                  beta = np.arctan(self.lr / self.L * np.tan( -using_state.steer / 180 * np.pi))
                  if beta != 0 :
                        R = self.lr / np.sin(beta)
                        yaw_rate = using_state.speed / R
                        delta_phi = delta_t * yaw_rate
                        alpha = delta_phi/2+beta
                        ld = np.absolute(2*R*np.sin(alpha - beta))
                        x_prime = updated_vector_point.x - ld * np.cos(alpha)
                        y_prime = updated_vector_point.y + ld * np.sin(alpha)
                        updated_vector_point.x = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                        updated_vector_point.y = -np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime

                  else :
                        updated_vector_point.x = updated_vector_point.x - using_state.speed * delta_t

                  #time indicator update
                  if next_time > _current_t or next_time == None:
                        time_indicator = _current_t
                  else:
                        time_indicator = next_time

            if is_exit:
                  is_success = True
            self.accessing_state -= 1
            return updated_vector_point, is_success


      def update_con_pid(self) : # final decision for control outputs
            if self.control_mode == Control_mode.EMERGENCY_BRAKE:
                  return
            current_state,current_state_time, next_time = self.get_current_state(rospy.Time.now().to_sec())
            if current_state == None:
                  return
            curspd = current_state.speed
            curstr = current_state.steer
            objspd = self.control.speed
            spderr = objspd - curspd

            self.control.speed = objspd + spderr * self.p_gain #adding p gain
            if self.control.speed < 0 :
                  self.control.speed = 0
            if self.control.speed > self.speed_max:
                  self.control.speed = self.speed_max

            objstr = self.control.steer
            strerr = objstr - curstr
            self.control.steer = objstr + strerr * self.p_gain
            if objspd < curspd : #TODO consider braking condition
                  self.control.brake = min(self.brake_adjust_max, spderr * self.brake_slope)




      def update_lpf (self) :
            alpha = 1.0/self.pub_rate / (1.0/self.pub_rate + 1.0/self.lpf_dt_cutoff)

            if len(self.control_buff) > 0 :
                  self.control.speed = (1-alpha) * self.control_buff[-1].speed + alpha * self.control.speed
                  self.control.steer = (1-alpha) * self.control_buff[-1].steer + alpha * self.control.steer


      def update_spath(self, _current_t):
            self.accessing_sPath += 1

            self.updated_sPath = PathArray()
            if len(self.sPath.pathpoints) == 0:
                  print "length of sPath is zero"
                  return False

            self.updated_sPath.header.stamp = self.sPath.header.stamp
            _past_t = self.sPath.header.stamp.to_sec()
            for i in self.sPath.pathpoints :
                  pathpoint_i = Vector3()
                  pathpoint_i.x = self.map_resolution*(self.map_height-i.y) #0.03m per pixel
                  pathpoint_i.y = self.map_resolution*(self.map_width/2-i.x)
                  pathpoint_i, is_transform_success = self.transform_pathpoint(pathpoint_i, _past_t, _current_t)
                  #JUNHO: transform_pathpoint is the new update_goalpoint
                  #IMPORTANT: is_transform_success: False when transform fails due to short state buffer
                  # self.update_goalpoint(st, t) #####################
                  if not is_transform_success:
                        print "transform failed in spath updating"
                        return False
                  v = Vector3()
                  v.z = 0
                  v.y = self.map_height - pathpoint_i.x / self.map_resolution
                  v.x = self.map_width/2 - pathpoint_i.y / self.map_resolution

                  if v.y < 200 :
                        self.updated_sPath.pathpoints.append(v)

            self.accessing_sPath -= 1
            return True

      #TODO: we have to check if cpoint updates well
      def update_cpoint(self, _current_t):
            _past_t = self.cPoint.header.stamp.to_sec()
            _goalpoint = Vector3()
            _goalpoint.x = self.cPoint.x_waypoint - self.cencorr
            _goalpoint.y = self.cPoint.y_waypoint
            _goalpoint, is_transform_success = self.transform_pathpoint(_goalpoint, _past_t, _current_t)
            #TODO fix this error
            if not is_transform_success:
                        # print "transform failed in cpoint updating"
                        #return False
                self.updated_cPoint.x_waypoint = self.cPoint.x_waypoint - self.cencorr
                self.updated_cPoint.y_waypoint = self.cPoint.x_waypoint - self.cencorr
            else:
                self.updated_cPoint.x_waypoint = _goalpoint.x
                self.updated_cPoint.y_waypoint = _goalpoint.y
            self.updated_cPoint.confidence = self.cPoint.confidence
            return True

      def main_control_loop(self) :

            '''
            1. Initialization
            '''
            self.control = Control()
            self.control_mode = Control_mode.NO_CONTROL
            self.control.is_auto = True
            self.control.estop = 0
            self.control.gear = 0 #0: front 1: neutral 2: rear
            self.control.speed = 0
            self.control.steer = 0 #in degree
            self.control.brake = 1

            current_t = rospy.Time.now().to_sec()

            '''
            2. Updating the s-path and c-point according to vehicle odometry
            '''
            #update s-path
            #erase s-path to empty object after a certain period of time
            if current_t - self.sPath.header.stamp.to_sec() > self.spath_time_threshold:
                  self.sPath.pathpoints = []
                  self.updated_sPath.pathpoints = []

            is_updated_spath= self.update_spath(current_t)
            if Z_DEBUG and not is_updated_spath :
                  print "s-path is not updated and is empty"
            #update c-point
            is_updated_cpoint = self.update_cpoint(current_t)
            # if Z_DEBUG and not is_updated_cpoint :
            #       print "c-point is not updated due to short state buffer"

            '''
            3. Deciding the control mode
            IMPORTANT: This is the most upper level logic of low-level controller!!
            '''
            # emergency stop case, emergency stop occurs by the e_stop message
            if current_t - self.estoptime < self.emergency_stop_keep :
                  if Z_DEBUG:
                        print "control mode: emergency braking"
                  self.control_mode = Control_mode.EMERGENCY_BRAKE

            # normal s path tracking, when the updated_sPath is still long enough to follow, the vehicle follows it.
            # This situation occurs during inside missions with rubber cones, and also when the vehicle is exiting the mission.
            elif len(self.updated_sPath.pathpoints) > self.spath_length_threshold :
                  if Z_DEBUG:
                        print "control mode: Normal S path tracking"
                  self.control_mode = Control_mode.NORMAL_S

            # c point tracking with low speed
            # This situation occurs when sPath is not created but there are obstacles
            # Usually occurs when entering into the mission with rubber cones, or when the sPath keeps fail updating during the mission.
            elif self.flag_obstacle > 0:
                  if Z_DEBUG:
                        print "control mode: C point tracking with low speed, there still are obstacles."
                  self.control_mode = Control_mode.SLOW_C

            else: #when flag_obstacle == 0
                  if Z_DEBUG:
                        print "control mode: Normal C point tracking"
                  self.control_mode = Control_mode.NORMAL_C
            #TODO add cases for uturn and parking control
            #TODO add case to deal with when cPoint is also empty

            '''
            4. Decide steering angle depending on each control mode
                  1) updtate goalpoint
                  if control mode is NORMAL_S, decide goalpoint according to sPath_n and updated_sPath
                  if control mode is NORMAL_C or SLOW_C, decide goalpoint according and updated_cPoint
                  2) pure pursuit logic
            '''
            # 1) update goalpoint
            if self.control_mode == Control_mode.NORMAL_S:
                  if len(self.updated_sPath.pathpoints) <= self.sPath_n :
                        self.goalpoint.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[-1].y)
                        self.goalpoint.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[-1].x)
                  else:
                        self.goalpoint.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[self.sPath_n].y)
                        self.goalpoint.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[self.sPath_n].x)
            if self.control_mode == Control_mode.NORMAL_C or self.control_mode == Control_mode.SLOW_C:
                        self.goalpoint.x_waypoint = self.updated_cPoint.x_waypoint
                        self.goalpoint.y_waypoint = self.updated_cPoint.y_waypoint
                        self.goalpoint.confidence = self.updated_cPoint.confidence
            # in other control_mode, goalpoint do not need to be set again because the control logic does not depend on the goalpoint.

            # 2) deciding PURE PURSUIT OUTPUT based on goalpoint
            if self.control_mode == Control_mode.NORMAL_S or self.control_mode == Control_mode.NORMAL_C or self.control_mode == Control_mode.SLOW_C:
                  ld = math.sqrt(self.goalpoint.x_waypoint*self.goalpoint.x_waypoint + self.goalpoint.y_waypoint*self.goalpoint.y_waypoint)
                  if ld == 0:
                        delta  = 0
                  else:
                        delta = -np.arctan(2 * self.L * self.goalpoint.y_waypoint / ld / (ld + 2 * self.lr * self.goalpoint.x_waypoint/ld)) # ld is lookahead distance
                  self.control.steer = delta / np.pi * 180 #in degree
            else:
                  self.control.steer = 0
            #TODO: set steer for u-turn and reverse


            '''
            5. Decide speed depending on each control mode
            '''
            if self.control_mode == Control_mode.NORMAL_S or self.control_mode == Control_mode.SLOW_C:
                  self.control.speed = self.speed_min
            elif self.control_mode == Control_mode.NORMAL_C:
                  self.control.speed = self.goalpoint.confidence * self.speed_slope
                  print(self.goalpoint.confidence)
            elif self.control_mode == Control_mode.NO_CONTROL:
                  self.control.speed = 0
            #TODO: set speed for u-turn and reverse
            elif self.control_mode == Control_mode.EMERGENCY_BRAKE:
                  self.control.speed = 0
                  self.control.brake = self.brake_max


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
            return self.control #need to publish self.control as control message

            #return to access free mode
            #exit

      #function that write, show, get the data of class
      def write_em(self) :
            self.estoptime = rospy.Time.now().to_sec()

      def write_cPoint(self, data) :
            self.cPoint=data
            self.ctime=rospy.Time.now().to_sec()

      def write_sPath(self, data) :
            #make sure that writing new sPath does not occurs during updating sPath
            # while self.accessing_sPath >= 1 :
            #       rospy.Rate(self.access_wait_rate).sleep()
            if len(data.pathpoints) > 0 :
                  self.sPath=data

      def write_stbuff(self, data) :
            self.writing_state = 1
            self.state_update_time = rospy.Time.now().to_sec()
            if len(self.state_buff) >= self.state_buff_size:
                  self.state_buff[0:-1]=self.state_buff[1:]
                  self.state_buff[-1]=data
                  self.state_time_buff[0:-1] = self.state_time_buff[1:]
                  self.state_time_buff[-1] = self.state_update_time
            else:
                  self.state_buff.append(data)
                  self.state_time_buff = np.append(self.state_time_buff, self.state_update_time)
            # if Z_DEBUG:
            #       print "state buffer size: ", len(self.state_buff)
            self.writing_state = 0

      def show_sPath(self) :
            for i in self.sPath.pathpoints :
                  print i

      def show_cPoint(self) :
            print self.ctime
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

def callback_stbuff(data) :
      main_track.write_stbuff(data)
#      main_track.show_stbuff()
#      print "Got state_buff"

def callback_emergency(data) :
      if data.data == 1:
            main_track.write_em()

def callback_obstacle(data):
      main_track.flag_obstacle = data.data


def init() :

      rospy.init_node('path_tracker', anonymous=True)
      rospy.Subscriber('/sPath', PathArray, callback_s)
      rospy.Subscriber('/waypoints', CenPoint, callback_c)
      rospy.Subscriber('/vehicle_state', VehicleState, callback_stbuff)
      rospy.Subscriber('/emergency_stop', Int32, callback_emergency)
      rospy.Subscriber('/flag_obstacle', Int32, callback_obstacle)

      cont_pub = rospy.Publisher('/control', Control, queue_size=10)
      mon_path_pub = rospy.Publisher('/path_tracking', PathArray, queue_size=10)
      rate = rospy.Rate(main_track.pub_rate)

      #TODO: set vehicle parameters from yaml file

      while not rospy.is_shutdown() :
            cont_pub.publish(main_track.main_control_loop())
            mon_path_pub.publish(main_track.updated_sPath)
            rate.sleep()


if __name__ == '__main__':
      init()
