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

import cv2
import json
import time
import base64
import pathlib
import socketserver
import numpy as np
from pioneer_sdk import Pioneer, Camera
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
camera = Camera()

# Time interval for each command
COMMAND_TIME_SECONDS = 0.05
TIMER_STEPS = 4  # 200 ms
TIMER_COUNT = 0

SPEED_TURN = 1.0
SPEED_MOVE = 1.0
SPEED_VERT = 1.0

# Min-max RC values
min_v = 1300
max_v = 1700

# Min-max height in meters
min_h = 0.5
max_h = 2.0

# RC min max
RC_MIN = 1000
RC_NONE = 1500
RC_MAX = 2000
RC_MOVE_RANGE = 200
RC_TURN_RANGE = 500
RC_VERT_RANGE = 500

# Channel values (in some 'RC' values)
def_ch_1 = RC_NONE
def_ch_2 = RC_NONE
def_ch_3 = RC_NONE
def_ch_4 = RC_NONE
def_ch_5 = RC_MAX

ch_1 = def_ch_1
ch_2 = def_ch_2
ch_3 = def_ch_3
ch_4 = def_ch_4
ch_5 = def_ch_5

# Is used to activate timer, as timers can only be started once (during the script life cycle)
isControlling = True

# Is used to stop the server on 'exit' command
isServing = True
isLoggingHttp = False

def resetCh():
    global ch_1, ch_2, ch_3, ch_4, ch_5, def_ch_1, def_ch_2, def_ch_3, def_ch_4, def_ch_5
    ch_1 = def_ch_1
    ch_2 = def_ch_2
    ch_3 = def_ch_3
    ch_4 = def_ch_4
    ch_5 = def_ch_5

def controlDef():
    global TIMER_COUNT, TIMER_STEPS, isControlling, ch_1, ch_2, ch_3, ch_4, ch_5
    TIMER_COUNT = TIMER_COUNT + 1    
    if TIMER_COUNT >= TIMER_STEPS:
        TIMER_COUNT = 0
        if not isControlling: return        
        pioneer_mini.send_rc_channels(
            channel_1=ch_1,
            channel_2=ch_2,
            channel_3=ch_3,
            channel_4=ch_4,
            channel_5=ch_5
        )
        resetCh()   

def moveHorRCValue(directionSignum):
    global RC_NONE, RC_MOVE_RANGE, SPEED_MOVE
    return int(RC_NONE + directionSignum * RC_MOVE_RANGE * SPEED_MOVE)

def moveVertRCValue(directionSignum):
    global RC_NONE, RC_VERT_RANGE, SPEED_VERT
    return int(RC_NONE + directionSignum * RC_VERT_RANGE * SPEED_VERT)

def turnRCValue(directionSignum):
    global RC_NONE, RC_TURN_RANGE, SPEED_TURN
    return int(RC_NONE + directionSignum * RC_VERT_RANGE * SPEED_VERT)

def limitValue(value, min, max):
    if value < min: return min
    if value > max: return max
    return value

pioneer_timer = RepeatTimer(COMMAND_TIME_SECONDS, controlDef)
pioneer_timer.start()

class WebRequestHandler(BaseHTTPRequestHandler):
    @cached_property
    def url(self):
        return urlparse(self.path)

    @cached_property
    def query_data(self):
        return dict(parse_qsl(self.url.query))
    
    def log_message(self, format, *args):
        global isLoggingHttp
        if isLoggingHttp:
            BaseHTTPRequestHandler.log_message(self, format, *args)

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
        global SPEED_MOVE, SPEED_TURN, SPEED_VERT, TIMER_STEPS, isControlling, isServing, ch_1, ch_2, ch_3, ch_4, ch_5, max_h, min_h, pioneer_mini, pioneer_timer
        data = self.query_data
       
        out = { "result": False }
        connected = pioneer_mini.connected()
        out["connected"] = connected
        action = ''
        d = pioneer_mini.get_dist_sensor_data(True)
        if d is None: d = 0 
        if 'action' in data: action = data['action']
        out["action"] = action
        action = str(action).lower()
        out["d"] = d
        if 'battery' in data:             
            out['battery'] = pioneer_mini.get_battery_status()
        if not connected:
            if action == "exit":
                pioneer_mini.land()
                isServing = False          
                out["p"] = "t"
                out["result"] = True
            else: 
                out["result"] = False
        if action == "settimervalue" and 'value' in data:
            TIMER_STEPS = int(limitValue(int(data['value']), 100, 1000) / (COMMAND_TIME_SECONDS * 1000))
        elif action == "setspeedmove" and 'value' in data: SPEED_MOVE = limitValue(float(data['value']), 0.1, 1.0)
        elif action == "setspeedturn" and 'value' in data: SPEED_TURN = limitValue(float(data['value']), 0.1, 1.0)
        elif action == "setspeedvert" and 'value' in data: SPEED_VERT = limitValue(float(data['value']), 0.1, 1.0)
        elif connected:
            out["result"] = True            
            if action == "frame":
                frame = camera.get_frame()
                if frame is not None:
                    img = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
                    img_enc = cv2.imencode(".jpg ", img)                    
                    img_str = img_enc[1].tobytes()                    
                    img_byte = base64.b64encode(img_str).decode("utf-8")
                    out["frame"] = img_byte
            elif action == "disarm": pioneer_mini.disarm()
           
            elif action == "left": resetCh(); ch_4 = moveHorRCValue(-1) 
            elif action == "right": resetCh(); ch_4 = moveHorRCValue(+1)
            elif action == "forward": resetCh(); ch_3 = moveHorRCValue(-1)
            elif action == "back": resetCh(); ch_3 = moveHorRCValue(+1)
            elif action == "turnleft": resetCh(); ch_2 = turnRCValue(+1) 
            elif action == "turnright": resetCh(); ch_2 = turnRCValue(-1) 
            elif action == "up":
                resetCh()
                if d < max_h: ch_1 = moveVertRCValue(+1) 
            elif action == "down": 
                resetCh()
                if d > min_h: ch_1 = moveVertRCValue(-1)
            elif action == "liftoff" or action == "takeoff":    
                pioneer_mini.arm()
                time.sleep(1)                
                pioneer_mini.takeoff()
                isControlling = True
            elif action == "land":
                isControlling = False
                pioneer_mini.land()
                time.sleep(2)
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
            html = '<html><body>pioneer.html not found</body></html>'
            file = pathlib.Path('pioneer.html')
            if file.exists():
                with open('pioneer.html', encoding="utf-8") as f:
                    html = f.read()
            else:
                file = pathlib.Path('../pioneer.html')
                if file.exists():
                    with open('../pioneer.html', encoding="utf-8") as f:
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
print("Open HTML WebUI in browser: http://localhost:" + str(PORT))
print("Use commands:")
print("  TAKEOFF: http://localhost:" + str(PORT) + "?action=takeoff")
print("     LAND: http://localhost:" + str(PORT) + "?action=land")
print(" TURNLEFT: http://localhost:" + str(PORT) + "?action=turnleft")
print("TURNRIGHT: http://localhost:" + str(PORT) + "?action=turnright")
print("  FORWARD: http://localhost:" + str(PORT) + "?action=forward")
print("     BACK: http://localhost:" + str(PORT) + "?action=back")

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

