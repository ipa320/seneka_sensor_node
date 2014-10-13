from json import dumps
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
         self.send(dumps({"op":"subscribe","id":"bridge_response","topic":"/bridge_response"});

     def closed(self, code, reason=None):
         print code, reason
         self.connect()

     def received_message(self, m):
         print "received", m
         #TODO: parse
         data="abc True"
         ar = data.split(" ")
         req = ar[0]
         success = bool(ar[1])
         if self.on_result!=None:
			 self.on_result(req,success)

if __name__=="__main__":
     try:
         ws = GetLoggersClient('ws://127.0.0.1:9090/')	#adjust here
         ws.connect()
         while True: time.sleep(1)
     except KeyboardInterrupt:
         ws.close()
