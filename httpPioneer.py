# (C) 2023 Neurobotics
# Author: Dmitry Konyshev (d.konyshev@neurobotics.ru)

# Description:
# This script starts a http-server on port 41017 and accepts commands (case insensitive):
# takeoff, land, forward, back, left, right, turnLeft, turnRight, up, down, status, exit

# Usage:
# 1. Install pioneer-sdk: pip install pioneer-sdk
# 2. Run: python httpPioneer.py
# 3. Call commands via http requests, e.g. 'http://127.0.0.1:41017/?action=takeoff'
# 4. Or use web UI by opening 'http://127.0.0.1:41017' or 'pioneer.html' in a browser

import json
import time
import socketserver
from pioneer_sdk import Pioneer #, Camera
from threading import Timer
from functools import cached_property
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler
from urllib.parse import parse_qsl, urlparse

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

PORT = 41017

pioneer_mini = Pioneer()

# Time interval for each command
COMMAND_TIME_SECONDS = 0.5

# Min-max RC values
min_v = 1300
max_v = 1700

# Min-max height in meters
min_h = 0.5
max_h = 2.0

# Channel values (in some 'RC' values)
def_ch_1 = 1500
def_ch_2 = 1500
def_ch_3 = 1500
def_ch_4 = 1500
def_ch_5 = 2000

ch_1 = def_ch_1
ch_2 = def_ch_2
ch_3 = def_ch_3
ch_4 = def_ch_4
ch_5 = def_ch_5

# Is used to activate timer, as timers can only be started once (during the script life cycle)
isControlling = True

# Is used to stop the server on 'exit' command
isServing = True

def resetCh():
    global ch_1, ch_2, ch_3, ch_4, ch_5, def_ch_1, def_ch_2, def_ch_3, def_ch_4, def_ch_5
    ch_1 = def_ch_1
    ch_2 = def_ch_2
    ch_3 = def_ch_3
    ch_4 = def_ch_4
    ch_5 = def_ch_5

def controlDef():
    global isControlling, ch_1, ch_2, ch_3, ch_4, ch_5
    # Sets controls for COMMAND_TIME_SECONDS and then resets them
    if isControlling:
        pioneer_mini.send_rc_channels(
            channel_1=ch_1,
            channel_2=ch_2,
            channel_3=ch_3,
            channel_4=ch_4,
            channel_5=ch_5
        )
        resetCh()


pioneer_timer = RepeatTimer(COMMAND_TIME_SECONDS, controlDef)
pioneer_timer.start()

class WebRequestHandler(BaseHTTPRequestHandler):
    @cached_property
    def url(self):
        return urlparse(self.path)

    @cached_property
    def query_data(self):
        return dict(parse_qsl(self.url.query))

    @cached_property
    def post_data(self):
        content_length = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(content_length)

    @cached_property
    def form_data(self):
        return dict(parse_qsl(self.post_data.decode("utf-8")))

    @cached_property
    def cookies(self):
        return SimpleCookie(self.headers.get("Cookie"))

    def get_response(self):
        global isServing, isControlling, ch_1, ch_2, ch_3, ch_4, ch_5, max_h, min_h, pioneer_mini, pioneer_timer
        data = self.query_data
       
        out = { "result": False }
        connected = pioneer_mini.connected()
        out["connected"] = connected
        action = ''
        d = pioneer_mini.get_dist_sensor_data(True)
        if d is None: d = 0 
        if 'action' in data: action = data['action']
        out["action"] = str(action).lower()
        out["d"] = d
        if not connected:
            if action == "exit":
                pioneer_mini.land()
                isServing = False          
                out["p"] = "t"
                out["result"] = True
            else: 
                out["result"] = False
        if connected:
            out["result"] = True
            if action == "left": resetCh(); ch_4 = min_v 
            elif action == "right": resetCh(); ch_4 = max_v 
            elif action == "forward": resetCh(); ch_3 = min_v 
            elif action == "back": resetCh(); ch_3 = max_v 
            elif action == "turnleft": resetCh(); ch_2 = 2000 
            elif action == "turnright": resetCh(); ch_2 = 1000 
            elif action == "up":
                resetCh()
                if d < max_h: ch_1 = 2000
            elif action == "down": 
                resetCh()
                if d > min_h: ch_1 = 1000
            elif action == "liftoff" or action == "takeoff":    
                pioneer_mini.arm()
                time.sleep(1)                
                pioneer_mini.takeoff()
                isControlling = True
            elif action == "land":
                isControlling = False
                pioneer_mini.land()
                time.sleep(1)
                pioneer_mini.disarm()
            elif action == "status":                   
                if connected:
                    out["d"] = d                    
            else:
                out["result"] = False
                out["error"] = "Provide valid action"
        return json.dumps(out)

    def do_GET(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')

        if len(self.query_data) == 0:
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            with open('pioneer.html', encoding="utf-8") as f:
                html = f.read()
                self.wfile.write(str(html).encode("utf-8"))
        else:
            self.send_header("Content-Type", "application/json")        
            self.end_headers()
            self.wfile.write(self.get_response().encode("utf-8"))
        

    def do_POST(self):
        if self.post_data:
            postJson = json.loads(self.post_data)
            for key in postJson:
              self.query_data[key] = postJson[key]
        self.do_GET()

Handler = WebRequestHandler

httpd = socketserver.TCPServer(("", PORT), Handler)
print("Serving at port", PORT)

def checkServer():
    global isServing, httpd
    if not isServing:
        httpd.shutdown()

# This timer handles stopping of server
timer = RepeatTimer(1, checkServer)

def main():    
    timer.start()
    httpd.serve_forever()
    #Server is working until 'isServing' is true

    timer.cancel()
    pioneer_timer.cancel()    

    pioneer_mini.land()
    time.sleep(1)
    pioneer_mini.disarm()
    print('Finished')

if __name__ == "__main__":
    main()

