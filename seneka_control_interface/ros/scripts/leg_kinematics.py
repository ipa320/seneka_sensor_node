#!/usr/bin/env python
#################################################################
##\file
#
# \note
#   Copyright (c) 2014 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n
#
#   All rights reserved. \n\n
#
#################################################################
#
# \note
#   ROS package name: ipa_kuka_rsi
#
# \author
#   Author: Joshua Hampp
#
# \date Date of creation: 06/21/2014
#
# \brief
#   test script
#
#################################################################

import roslib; roslib.load_manifest('seneka_control_interface')
import rospy
import actionlib

from threading import Thread
import collections, copy, math

import sensor_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import std_srvs.srv
import seneka_control_interface.msg
import laser_assembler.srv

class StateMachine:
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise InitializationError("must call .set_start() before .run()")
        if not self.endStates:
            raise  InitializationError("at least one state must be an end_state")
    
        while True:
            (newState, cargo) = handler(cargo)
            if newState.upper() in self.endStates:
                print("reached ", newState)
                break 
            else:
                handler = self.handlers[newState.upper()]

class Comm:
	def __init__(self):
		#configuration
		self.kin_extend = rospy.get_param('~kinematics_extend')
		self.kin_retract = rospy.get_param('~kinematics_retract')
		self.joint_turret = rospy.get_param('~turrent_joint')
		self.joint_camera = rospy.get_param('~camera_joint')
		
		#ros stuff
		self.last_pos = {}
		self.pub_pc = rospy.Publisher('/laser_pc', sensor_msgs.msg.PointCloud)
		self.pub_resp = rospy.Publisher('bridge_response', std_msgs.msg.String)
		rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.on_joint_state)
		#rospy.wait_for_service('assemble_scans')
		self.srv_assemble_scans = rospy.ServiceProxy("assemble_scans", laser_assembler.srv.AssembleScans)
		self.buttons = []
		for i in xrange(3):
			self.buttons.append(False)
			#rospy.Subscriber("button"+str(i), std_msgs.msg.Bool, self.on_button, i)
		rospy.Service('extend', std_srvs.srv.Empty, self.extend)
		rospy.Service('retract', std_srvs.srv.Empty, self.retract)
		rospy.Service('scan', std_srvs.srv.Empty, self.scan)
		rospy.Subscriber("move_turret", std_msgs.msg.Float64, self.on_turret_aim)
		rospy.Subscriber("move_camera", std_msgs.msg.Float64, self.on_camera_aim)
		
	def send_response(self, msg, success=True):
		msg = std_msgs.msg.String()
		msg.data = str(msg)+" "+str(success)
		self.pub_resp.publish(msg)

	def on_turret_aim(self, msg):
		r = self.send_kinematics([[msg.data]],[self.joint_turret], False)
		self.send_response("move_turret", r)

	def on_camera_aim(self, msg):
		r = self.send_kinematics([[msg.data]],[self.joint_camera], False)
		self.send_response("move_camera", r)
		
	def on_joint_state(self, msg):
		if len(msg.name)!=len(msg.position): return
		for i in xrange(len(msg.name)):
			self.last_pos[msg.name[i]] = msg.position[i]
		
	def extend(self, _dummy):
		r = self.exec_srv(self.kin_extend)
		self.send_response("extend", r)
		return r

	def retract(self, _dummy):
		param = copy.deepcopy(self.kin_retract)
		for l in param: #l=legX
			if not l+"0" in self.last_pos:
				print "not found "+l
				continue
			pos = self.last_pos[l+"0"]
			val = param[l]
			print pos, val
			for i in range(len(val)-1):
				if pos>=min(val[i][0],val[i+1][0]) and pos<=max(val[i][0],val[i+1][0]):
					print "found at "+str(i)
					for j in range(i+1): param[l].pop(0)
					break
		r = self.exec_srv(param)
		self.send_response("retract", r)
		return r
	
	def send_kinematics(self, traj, name, check):
		client = actionlib.SimpleActionClient('/ex_joint_trajectory', seneka_control_interface.msg.JointTrajectoryAction)
		client.wait_for_server()

		msg = trajectory_msgs.msg.JointTrajectory()
		msg.joint_names = name
		
		if len(traj)>0 and isinstance(traj[0], collections.Iterable):
			for t in traj:
				pt = trajectory_msgs.msg.JointTrajectoryPoint()
				pt.positions = t
				pt.time_from_start = rospy.rostime.Duration(30)
				msg.points.append(pt)
		else:
			pt = trajectory_msgs.msg.JointTrajectoryPoint()
			pt.positions = traj
			pt.time_from_start = rospy.rostime.Duration(10)
			msg.points.append(pt)
		
		req = seneka_control_interface.msg.JointTrajectoryGoal()
		req.traj = msg
		req.check_switch = [check]*len(pt.positions)
		print req

		client.send_goal(req)
		client.wait_for_result(rospy.Duration.from_sec(50.0))
		return True

	def on_button(self, msg, index):
		self.buttons[index] = msg.data
		
	def execute_kinematic(self, name, traj):
		for t in traj:
			check = False
			pos = t[0:2]
			if len(t)>2: check = t[2]
			if not isinstance(pos, collections.Iterable): pos = [pos]
			r = self.send_kinematics(pos, name, check)
			if r and check: return True
			elif not r: return False
		return True

	def exec_srv(self, param):
		threads=[]
		for name in param:
			t = Thread(target=self.execute_kinematic, args=(param[name].joints, param[name].kin))
			t.start()
			threads.append( t )
		for t in threads:
			t.join()
		return std_srvs.srv.EmptyResponse()
		
	def scan(self, _dummy):
		req=laser_assembler.srv.AssembleScansRequest()
		req.begin = rospy.Time.now()
		kin = [[0], [math.pi], [2*math.pi-0.05]]
		# use shortest path
		if self.joint_turret in self.last_pos and self.last_pos[self.joint_turret]>math.pi:
				kin.reverse()
		r = self.send_kinematics(kin,[self.joint_turret], False)
		req.end = rospy.Time.now()
		
		resp = self.srv_assemble_scans(req)
		self.pub_pc.publish(resp.cloud)
		
		self.send_response("scan", r)
		
		return std_srvs.srv.EmptyResponse()


if __name__ == '__main__':
    try:
        rospy.init_node('leg_kinematics')
	comm = Comm()
	rospy.spin()

	#rospy.sleep(10.1)
	#comm.send_kinematics([[0,1,2],[3,4,5]])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
