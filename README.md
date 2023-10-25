# pioneer-server
A python-based server and HTML webUI to control [Geoscan Pioneer Mini](https://www.geoscan.ru/ru/pioneer-mini) drones (for neurointerface control)

## Description
This script starts a http-server on port 41017 and accepts commands (case insensitive):  
takeoff, land, forward, back, left, right, turnLeft, turnRight, up, down, status, exit

## Usage:
1. Install pioneer-sdk:

        pip install pioneer-sdk
2. Run:
   
        python httpPioneer.py
3. Call commands via http requests, e.g. 'http://127.0.0.1:41017/?action=takeoff'
4. Or use web UI by opening 'http://127.0.0.1:41017' or 'pioneer.html' in a browser

## BCI usage (NeuroPlay neurointerface):
1. Open the mental states tab in [NeuroPlayPro](https://neuroplay.ru) app
2. Check the Actions drop down
3. Use HTTP-type and write in desired mental state actions: "http://127.0.0.1:41017/?action=forward", "http://127.0.0.1:41017/?action=turnLeft", etc.
