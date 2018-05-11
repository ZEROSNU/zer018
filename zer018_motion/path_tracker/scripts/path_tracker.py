#!/usr/bin/env python
import rospy
import numpy as np
import math
from core_msgs.msg import PathArray
from core_msgs.msg import Control
from core_msgs.msg import CenPoint
from core_msgs.msg import VehicleState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

Z_DEBUG = True

class tracker :
      def __init__ (self) :
            self.pub_rate = 20.0 #Hz
            self.access_wait_rate = 500 #Hz
            self.buff_size = 100
            self.speed_slope = 0.2 #m/s/conf
            self.delta_y_max = 0.3 #m maximum error
            self.speed_max = 2.1 #m/s
            self.speed_min = 0.5
            self.speed_plan = 0.0
            self.speed_delta = 0.1
            self.p_gain = 0.3
            self.emergency_stop_keep = 10 #sec
            self.map_resolution = 0.03
            self.s_response = 10 #se TODO match with buff_size
            self.brake_max = 200
            self.brake_slope = 50
            self.lpf_dt_cutoff = 10
            self.use_s_thres = 10
            # -------------^CONSTANT^--------------

            self.L = 1.54 #m distance between two wheel axis
            self.lr = 0.53 #m  distance between cm and rear wheel
            self.cencorr = -0.04 #m  origin distance between cenPoint and sPath
            #car coordinate origin will be the geometrical center of ca0r frame, except bumper.
            #goalpoint should be cenPoint.y_waypoint + self.cencorr

            # ----------------------------------^HARDWARE_CONSTANT^------------------------------


            self.control = Control() #my order to the car
            self.goalpoint = CenPoint() #data of goal point - sPath origin placed in center, cenPath origin placed 770 from front wheel
            self.cont_buff = np.empty(self.buff_size, dtype=(type([Control(), float()]))) #TODO change this data structure
            self.updated_sPath = PathArray()
            #time variables _ time when this node get sPath or cenPoint
            self.stime = 0
            self.ctime = 0
            self.sttime = 0
            self.estoptime = -10
            self.accessing_state = 0
            self.accessing_sPath = 0
            self.writing_state = 0
            self.writing_sPath = 0
            #-----------------------^DETERMINED^---------------------

            self.sPath = PathArray()
            self.cPoint = CenPoint()
            self.sDelay = 0.0
            self.cDelay = 0.0
            #######################TODO change this to fill default state
            self.state_buff = np.empty(self.buff_size, dtype=type([VehicleState(), float()]))
            self.sPath_n = 0
            #-------^INPUT^---------------

      #function needs to update goalpoint - get speed and steering at time t
      def get_state(self, time) :
            t_state = VehicleState()
            while self.writing_state > 0 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.accessing_state += 1
            # if self.state_buff[self.buff_size-1] == None : #TODO fix this
            #       print "no data in state_buff"
            #       self.accessing_state -= 1
            #       return t_state
            t_state.is_auto = self.state_buff[self.buff_size-1][0].is_auto
            t_state.estop = self.state_buff[self.buff_size-1][0].estop
            t_state.brake = self.state_buff[self.buff_size-1][0].brake
            t_state.gear = self.state_buff[self.buff_size-1][0].gear
            t_state.speed = self.state_buff[self.buff_size-1][0].speed
            t_state.steer = self.state_buff[self.buff_size-1][0].steer
            t_state.encoder = self.state_buff[self.buff_size-1][0].encoder
            t_state.alive = self.state_buff[self.buff_size-1][0].alive
            #TODO change this logic to numpy masking
            for i in reversed(self.state_buff) :
                  if i == None :
                        break
                  elif time < i[1] :
                        t_state.is_auto = i[0].is_auto
                        t_state.estop = i[0].estop
                        t_state.gear = i[0].gear
                        t_state.brake = i[0].brake
                        t_state.speed = i[0].speed
                        t_state.steer = i[0].steer
                        t_state.encoder = i[0].encoder
                        t_state.alive = i[0].alive
                  else :
                        break
            self.accessing_state -= 1
            return t_state

      def update_con_pid(self) :
            curspd = self.get_state(rospy.Time.now().to_sec()).speed
            curstr = self.get_state(rospy.Time.now().to_sec()).steer
            objspd = self.control.speed
            spderr = objspd - curspd
            self.control.speed = objspd + spderr * self.p_gain


            objstr = self.control.steer
            strerr = objstr - curstr
            self.control.steer = objstr + strerr * self.p_gain
            if objspd < curspd : #TODO consider braking condition
                  self.control.brake = spderr * self.brake_slope



      def update_lpf (self) :
            alpha = 1.0/self.pub_rate / (1.0/self.pub_rate + 1.0/self.lpf_dt_cutoff)
            
            if len(self.cont_buff) > 0 : #TODO change data structure of control buffer
                  self.control.speed = (1-alpha) * self.cont_buff[self.buff_size - 1][0].speed + alpha * self.control.speed
                  self.control.steer = (1-alpha) * self.cont_buff[self.buff_size - 1][0].steer + alpha * self.control.steer




      def update_goalpoint(self, intime, curtime) : # the time when the goalpoint was input
            ugoalpoint = CenPoint()
            ugoalpoint.x_waypoint = self.goalpoint.x_waypoint
            ugoalpoint.y_waypoint = self.goalpoint.y_waypoint
            ugoalpoint.confidence = self.goalpoint.confidence
            time_ind = intime
            init = False
            tmp = [VehicleState(), float()]
            while self.writing_state > 0 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.accessing_state += 1
            ###################################################################################
            #check state_buffer empty
            for ind in self.state_buff :        #state_buff[state_buff>time_ind]
                  #if ind == None : ###TODO fix None
                  #      continue
                  elif ind[1] - time_ind >=0 :
                        if init == False :
                              beta = np.arctan(self.lr / self.L * np.tan( -self.get_state(time_ind).steer / 180 * np.pi))
                        else :
                              beta = np.arctan(self.lr / self.L * np.tan( -tmp[0].steer / 180 * np.pi))
                        if beta != 0 :
                              R = self.lr / np.sin(beta)
                              if init == False :
                                    yaw_rate = self.get_state(time_ind).speed / R
                              else :
                                    yaw_rate = tmp[0].speed / R
                              delta_t = time_ind - ind[1]
                              delta_phi = delta_t * yaw_rate
                              alpha = delta_phi/2+beta
                              ld = np.absolute(2*R*np.sin(alpha - beta))
                              x_prime = ugoalpoint.x_waypoint - ld * np.cos(alpha)
                              y_prime = ugoalpoint.y_waypoint - ld * np.sin(alpha)
                              ugoalpoint.x_waypoint = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                              ugoalpoint.y_waypoint = -np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime

                        else :
                              if init == False :
                                    ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - self.get_state(time_ind).speed * (time_ind - ind[1])
                              else :
                                    ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - tmp[0].speed * (time_ind - ind[1])

                        time_ind = ind[1]
                        tmp[0] = ind[0]
                        tmp[1] = ind[1]
                        init = True
            

            if init == False :
                  self.accessing_state -= 1
                  return

            beta = np.arctan(self.lr / self.L * np.tan( -self.get_state(time_ind).steer / 180 * np.pi))
            if beta!=0 :
                  R = self.lr / np.sin(beta)
                  yaw_rate = self.get_state(curtime).speed / R
                  delta_t = curtime - ind[1]
                  delta_phi = delta_t * yaw_rate
                  alpha = delta_phi/2+beta
                  ld = np.absolute(2*R*np.sin(alpha - beta))
                  x_prime = ugoalpoint.x_waypoint - ld * np.cos(alpha)
                  y_prime = ugoalpoint.y_waypoint - ld * np.sin(alpha)
                  ugoalpoint.x_waypoint = np.cos(delta_phi) * x_prime + np.sin(delta_phi) * y_prime
                  ugoalpoint.y_waypoint = -np.sin(delta_phi) * x_prime + np.cos(delta_phi) * y_prime
            else :
                  ugoalpoint.x_waypoint = ugoalpoint.x_waypoint - self.get_state(curtime).speed * (time_ind - ind[1])
            
            self.goalpoint.x_waypoint = ugoalpoint.x_waypoint
            self.goalpoint.y_waypoint = ugoalpoint.y_waypoint
            self.accessing_state -= 1
      #how to update goalpoint integrate state_buff 0 to n-1

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



      #need to decide n for s_path




      def calc_get_cont(self) :
            self.control = Control()
            self.control.is_auto = True
            self.control.estop = 0
            self.control.gear = 0
            self.control.speed = 0
            self.control.steer = 0 #in degree
            self.control.brake = 1
            self.accessing_sPath = 0
            self.accessing_state = 0
            while self.writing_sPath > 0 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.accessing_sPath += 1
            pttype = 'n'
            t = rospy.Time.now().to_sec()
            st = self.stime
            self.updated_sPath = PathArray()
            for i in self.sPath.pathpoints :

                  self.goalpoint.x_waypoint = self.map_resolution*(200-i.y) #0.3m per pixel
                  self.goalpoint.y_waypoint = self.map_resolution*(100-i.x)
                  self.update_goalpoint(st, t) ##################### TODO fix st to spath time stamp
                  v = Vector3()
                  v.z = 0
                  v.y = 200 - self.goalpoint.x_waypoint / self.map_resolution
                  v.x = 100 - self.goalpoint.y_waypoint / self.map_resolution

                  #if v.y < 200 :
                  self.updated_sPath.pathpoints.append(v)
            self.updated_sPath.header.stamp = self.sPath.header.stamp
            self.accessing_sPath -= 1
            

            if t - self.estoptime < 10 :
                  print "estop"
                  pttype = 'e'
                  self.control.speed = 0
                  self.control.brake = self.brake_max
                  self.cont_buff[0:-1]=self.cont_buff[1:self.buff_size]
                  self.cont_buff[self.buff_size-1] = [self.control, rospy.Time.now().to_sec()]
                  self.speed_plan = 0
                  return self.control

            #TODO merge control state control logic
            if self.stime>0 and len(self.updated_sPath.pathpoints) > self.use_s_thres and pttype == 'n' :
                  print 'publish based on sPath'
                  pttype = 's'
            # if self.decide_sPath_n(self.speed_plan) > self.delta_y_max :
                                          
            #       while self.decide_sPath_n(self.speed_plan) > self.delta_y_max :
            #             self.speed_plan -= self.speed_delta
            #             if self.speed_plan > self.speed_min :
            #                   print "speed should be much slower"
            #                   break
            #             elif self.decide_sPath_n(self.speed_plan + self.speed_delta) < self.delta_y_max :
            #                   self.speed_plan += self.speed_delta
            #             else :
            #                   self.decide_sPath_n(self.speed_plan)
                  self.sPath_n = 20 #############################
                  if len(self.updated_sPath.pathpoints) <= self.sPath_n :
                        self.sPath_n = len(self.updated_sPath.pathpoints)-1
                  self.speed_plan = self.speed_min
                  self.goalpoint.x_waypoint = self.map_resolution*(200-self.updated_sPath.pathpoints[self.sPath_n].y)
                  self.goalpoint.y_waypoint = self.map_resolution*(100-self.updated_sPath.pathpoints[self.sPath_n].x)
            if self.ctime>0 and pttype =='n': #######3 TODO change order of updating path and control
                  print "publish based on cPath"
                  pttype = 'c'
                  self.goalpoint.x_waypoint = self.cPoint.x_waypoint - self.cencorr
                  self.goalpoint.y_waypoint = self.cPoint.y_waypoint
                  self.goalpoint.confidence = self.cPoint.confidence
                  self.update_goalpoint(self.ctime, t)
                  self.speed_plan = self.goalpoint.confidence * self.speed_slope
                  # if tmpspd > self.speed_plan :
                  #       self.speed_plan += self.speed_delta
                  # elif tmpspd < self.speed_plan :
                  #       self.speed_plan -= self.speed_delta
            if pttype == 'n' :
                  print "no input"
                  return self.control

            """
            PURE PURSUIT OUTPUT
            """
            ld = math.sqrt(self.goalpoint.x_waypoint*self.goalpoint.x_waypoint + self.goalpoint.y_waypoint*self.goalpoint.y_waypoint)
            delta = -np.arctan(2 * self.L * self.goalpoint.y_waypoint / ld / (ld + 2 * self.lr * self.goalpoint.x_waypoint/ld)) # ld is lookahead distance



            self.control.speed = self.speed_plan
            self.control.steer = delta / np.pi * 180 #in degree

            self.cont_buff[0:-1]=self.cont_buff[1:self.buff_size]
            self.cont_buff[self.buff_size-1] = [self.control, rospy.Time.now().to_sec()]

            self.update_lpf()
            self.update_con_pid()

            if self.control.speed < 0 :
                  self.control.speed = 0
            return self.control #need to update self.control

      #need function for PID gain



      #function that write, show, get the data of class
      def write_em(self) :
            self.estoptime = rospy.Time.now().to_sec()


      def write_cPoint(self, data) :
            self.cPoint=data
            self.ctime=rospy.Time.now().to_sec()

      def write_sPath(self, data) :
            while self.accessing_sPath>1 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.writing_sPath += 1
            if len(data.pathpoints) > 0 :
                  self.sPath=data
                  self.stime=rospy.Time.now().to_sec()
            self.writing_sPath -= 1

      def write_stbuff(self, data) :
            while self.accessing_state>1 :
                  rospy.Rate(self.access_wait_rate).sleep()
            self.writing_state += 1
            self.sttime = rospy.Time.now().to_sec()
            self.state_buff[0:-1]=self.state_buff[1:self.buff_size]
            self.state_buff[self.buff_size-1]=[data, self.sttime]
            self.writing_state -= 1

      def show_sPath(self) :
            for i in self.sPath.pathpoints :
                  print i
            print self.stime

      def show_cPoint(self) :
            print self.ctime
            print self.cPoint

      def show_stbuff(self) :
            print self.state_buff

      def show_cont_buff(self) :
            for i in self.cont_buff :
                  print i

      def show_control(self) :
            print self.control

      def get_sPath(self) :
            return self.sPath

      def get_cPoint(self) :
            return self.cPoint


      def get_cont_buff(self, i) :
            return self.cont_buff[i]

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


def init() :

      rospy.init_node('path_tracker', anonymous=True)
      rospy.Subscriber('/sPath', PathArray, callback_s)
      rospy.Subscriber('/waypoints', CenPoint, callback_c)
      rospy.Subscriber('/vehicle_state', VehicleState, callback_stbuff)
      rospy.Subscriber('/emergency_stop', Int32, callback_emergency)

      cont_pub = rospy.Publisher('/control', Control, queue_size=10)
      mon_path_pub = rospy.Publisher('/path_tracking', PathArray, queue_size=10)

      rate = rospy.Rate(main_track.pub_rate)
      
      while not rospy.is_shutdown() :
            cont_pub.publish(main_track.calc_get_cont())
            mon_path_pub.publish(main_track.updated_sPath)
            rate.sleep()


if __name__ == '__main__':
      init()
