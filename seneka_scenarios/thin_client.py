from json import dumps
import json, sys
#https://github.com/Lawouach/WebSocket-for-Python
from ws4py.client.threadedclient import WebSocketClient
import time

class GetLoggersClient(WebSocketClient):
     #interface
     on_result = None

     def extend(self):
         self.call("/extend")

     def retract(self):
         self.call("/retract")

     def scan(self):
         self.call("/scan")
         
     def move_to(self, pos):
         msg = {'op': 'publish', 'topic': '/move_turret', 'msg': {'data': pos}}
         self.send(dumps(msg))

     #internal
     def call(self, srv):
         msg = {'op': 'call_service', 'service': srv}
         self.send(dumps(msg))

     def opened(self):
         print "Connection opened..."
         msg = {'op': 'subscribe', 'topic': "/bridge_response"}
         self.send(dumps(msg))

     def closed(self, code, reason=None):
         print code, reason
         self.connect()

     def received_message(self, m):
         try:
                  msg = json.loads(str(m))
                  if not 'topic' in msg or msg['topic']!='/bridge_response': return
         
                  ar = msg['msg']['data'].split(" ")
                  if len(ar)<2: return
                  req = ar[0]
                  success = bool(ar[1])
                  if self.on_result!=None:
                           self.on_result(req,success)
         except:
                  print "error", sys.exc_info()[0]

def sample_handler(req, success):
	print req, success
	
if __name__=="__main__":
     try:
         ws = GetLoggersClient('ws://127.0.0.1:9090/')	#adjust here
         ws.on_result = sample_handler
         ws.connect()
         while True: time.sleep(1)
     except KeyboardInterrupt:
         ws.close()
