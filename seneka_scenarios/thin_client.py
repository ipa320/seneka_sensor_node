
import roslib; roslib.load_manifest('seneka_scenarios')
import rospy

import std_msgs.msg
import sensor_msgs.msg
import std_srvs.srv

from json import dumps
import json, sys
#https://github.com/Lawouach/WebSocket-for-Python
from ws4py.client.threadedclient import WebSocketClient
import time

class GetLoggersClient(WebSocketClient):
     #interface
     on_result = None

     def extend(self, dummy=None):
         self.call("/extend")
         return std_srvs.srv.EmptyResponse()

     def retract(self, dummy=None):
         self.call("/retract")
         return std_srvs.srv.EmptyResponse()

     def scan(self):
         self.call("/scan")
         return std_srvs.srv.EmptyResponse()
         
     def move_to(self, pos):
         msg = {'op': 'publish', 'topic': '/move_turret', 'msg': {'data': pos}}
         self.send(dumps(msg))
         
     def on_move_to(self, msg):
         self.move_to(msg.data)

     #internal
     def call(self, srv):
         msg = {'op': 'call_service', 'service': srv}
         self.send(dumps(msg))

     def opened(self):
         print "Connection opened..."
         msg = {'op': 'subscribe', 'topic': "/bridge_response"}
         msg = {'op': 'subscribe', 'topic': "/joint_states", 'throttle_rate': 20}
         self.send(dumps(msg))

     def closed(self, code, reason=None):
         print code, reason
         self.connect()

     def received_message(self, m):
         try:
                  msg = json.loads(str(m))
                  if not 'topic' in msg: return
         
                  if msg['topic']=='/joint_states':
					  result = sensor_msgs.msg.JointState()
					  result.position = msg['msg']['position']
					  result.name = msg['msg']['name']
					  result.header.stamp.secs = msg['msg']['header']['stamp']['secs']
					  result.header.stamp.nsecs = msg['msg']['header']['stamp']['nsecs']
					  self.pub_joint_states.publish(result)
					  
                  if msg['topic']=='/bridge_response':
					  ar = msg['msg']['data'].split(" ")
					  if len(ar)<2: return
					  req = ar[0]
					  success = bool(ar[1])
					  result = std_msgs.msg.String()
					  result.data = msg['msg']['data']
					  self.pub_resp.publish(result)
					  if self.on_result!=None:
							   self.on_result(req,success)
         except:
                  print "error", sys.exc_info()[0]

def sample_handler(req, success):
	print req, success
	
if __name__=="__main__":
     try:
         rospy.init_node('bridge')
        
         ws = GetLoggersClient('ws://172.21.6.102:9090/')	#adjust here
         
         ws.on_result = sample_handler
         ws.pub_resp = rospy.Publisher('bridge_response', std_msgs.msg.String)
         ws.pub_joint_states = rospy.Publisher('/sensornode/joint_states', sensor_msgs.msg.JointState)
         rospy.Service('extend', std_srvs.srv.Empty, ws.extend)
         rospy.Service('retract', std_srvs.srv.Empty, ws.retract)
         rospy.Subscriber("move_turret", std_msgs.msg.Float64, ws.on_move_to)
         ws.connect()
         
         rospy.spin()
     except KeyboardInterrupt:
         ws.close()
