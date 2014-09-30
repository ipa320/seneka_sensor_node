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

from threading import Thread
import collections, copy, math

import sensor_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import std_srvs.srv
import seneka_srv.srv
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
		
		#ros stuff
		self.last_pos = {}
		self.pub_pc = rospy.Publisher('/laser_pc', sensor_msgs.msg.PointCloud)
		rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.on_joint_state)
		rospy.wait_for_service('joint_path_command')
		self.srv_traj = rospy.ServiceProxy("joint_path_command", seneka_srv.srv.JointTrajectory)
		rospy.wait_for_service('assemble_scans')
		self.srv_assemble_scans = rospy.ServiceProxy("assemble_scans", laser_assembler.srv.AssembleScans)
		self.buttons = []
		for i in xrange(3):
			self.buttons.append(False)
			#rospy.Subscriber("button"+str(i), std_msgs.msg.Bool, self.on_button, i)
		rospy.Service('extend', std_srvs.srv.Empty, self.extend)
		rospy.Service('retract', std_srvs.srv.Empty, self.retract)
		rospy.Service('scan', std_srvs.srv.Empty, self.scan)

	def on_joint_state(self, msg):
		if len(msg.name)!=len(msg.position): return
		for i in xrange(len(msg.name)):
			self.last_pos[msg.name[i]] = msg.position[i]
		
	def extend(self, _dummy):
		return self.exec_srv(self.kin_extend)

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
		return self.exec_srv(param)
	
	def send_kinematics(self, traj, name, check):
		msg = trajectory_msgs.msg.JointTrajectory()
		msg.joint_names = name
		
		if len(traj)>0 and isinstance(traj[0], collections.Iterable):
			for t in traj:
				pt = trajectory_msgs.msg.JointTrajectoryPoint()
				pt.positions = t
				pt.time_from_start = rospy.rostime.Duration(10)
				msg.points.append(pt)
		else:
			pt = trajectory_msgs.msg.JointTrajectoryPoint()
			pt.positions = traj
			pt.time_from_start = rospy.rostime.Duration(10)
			msg.points.append(pt)
		
		try:
			req = seneka_srv.srv.JointTrajectoryRequest()
			req.traj = msg
			req.check_switch = [check]*len(pt.positions)
			return self.srv_traj(req)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		return False

	def on_button(self, msg, index):
		self.buttons[index] = msg.data
		
	def execute_kinematic(self, name, traj):
		for t in traj:
			check = False
			pos = t[0:2]
			if len(t)>2: check = t[2]
			if not isinstance(pos, collections.Iterable): pos = [pos]
			r = self.send_kinematics(pos, [name+"0",name+"1"], check)
			if r and check: return True
			elif not r: return False
		return True

	def exec_srv(self, param):
		threads=[]
		for name in param:
			t = Thread(target=self.execute_kinematic, args=(name, param[name]))
			t.start()
			threads.append( t )
		for t in threads:
			t.join()
		return std_srvs.srv.EmptyResponse()
		
	def scan(self, _dummy):
		req=laser_assembler.srv.AssembleScansRequest()
		req.begin = rospy.Time.now()
		kin = [[0], [2*math.pi-0.01]]
		# use shortest path
		if "turret" in self.last_pos and self.last_pos["turret"]>math.pi:
				kin.reverse()
		self.send_kinematics(kin,["turret"], False)
		req.end = rospy.Time.now()
		
		resp = self.srv_assemble_scans(req)
		self.pub_pc.publish(resp.cloud)
		
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
