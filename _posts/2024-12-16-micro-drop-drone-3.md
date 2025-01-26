---
#layout: posts
title: Micro Drop Drone 3 - Software-In-The-Loop
date: 2024-12-16 13:00:00 +0530
categories:
  - micro drop drone
tags:
  - drones
  - low-gravity
---

## What is SITL?
ArduPilot comes with software-in-the-loop (SITL) capabilities. This is some code that runs a simulation of a drone, allowing you to test out scripts on a virtual or proxy drone. Within ArduPilot, there is a script called sim_vehicle.py that launches and runs a SITL simulation, and runs an instance of MAVProxy that connects to this proxy drone, allowing commands to be sent and telemetry read from the drone. A separate python script can also connect to the port of the proxy drone, allowing this proxy drone to be automated.

With this capability, we can test scripts that command a drone to fly a vertical parabolic manoeuvre using ArduPilot's SITL without needing to operate a real drone. Naturally, this speeds up development and minimizes risk. Since the drone I previously built is not needed yet, I've sent this back to the UK early.

## How to Run a SITL Instance
The following is a summary of how I set up and run SITL instances. If you need further guidance, I highly recommend [ArduPilot's website](https://ardupilot.org/dev/docs/where-to-get-the-code.html) and [Caleb's Drone Dojo videos](https://youtu.be/TO7qa8oCACI?si=LFwZP-_T11lm0z3m)

I use windows and while SITL can be run on windows, I decided to follow Caleb's guide and use Windows Subsystems for Linux (WSL2 to be exact) to become more familiar with Linux. I installed Ubuntu 24.04. I then cloned the ArudPilot git repository following [Caleb's guide](https://youtu.be/uvZIfGBqLPE?si=M6_7AldYhjrevKwq). I also followed [Caleb's video](https://youtu.be/CvzvLN23TjA?si=jF62TiO93YtuTPyp) to set up the work environment, making it easier to get into the ArduPilot folder each time.

To run an SITL instance, you need to navigate to the vehicle folder within the ArduPilot directory, e.g. ArduCopter to run a SITL instance for a quadcopter. Once there, running `sim_vehicle.py` will start an SITL instance. Adding `--map` and `--console` should also load a map and console to interact with this proxy drone. 

When trying to follow [Caleb's video](https://youtu.be/hNUoF7uGVl0?si=yFYiwHxoxSkc0vWt) to launch an SITL instance using `sim_vehicle.py`, I ran into a few issues. The first were a few missing dependencies such as `pexpect`, `future`, and `dronecan`. I installed these using `pip install <package>`. Even with all dependencies installed, the map and console if `sim_vehicle.py` would not load. Apparently one ArduPilot module (whose name I didn't record, sorry) was compiled using a version of numpy < 2.0, hence I needed to downgrade my numpy from 2.1.3 to \< 2.0. I downgraded to `numpy 1.26.0`. The console would then load. 

The map would not load due to an issue with `mavproxy.py` when launched by sim_vehicle.py. After running mavproxy.py on its own with `modedebug=3` (to see more debug info), I saw `openCv` needed installing. I then ran `sudo apt install python3-opencv`. The map module then loaded.

## Connecting Mission Planner to SITL
Mission Planner is an example of ground control software, a piece of software that can communicate with a drone. MAVProxy is also an example of ground control software, though the interface is not as clean or user-friendly as Mission Planner. With Mission Planner, you can connect to a drone (or proxy drone), define flight paths by clicking on a map, alter the drone's parameters, among other features. QGround Contol is another example of ground control software.

With Mission Planner  already running, starting an SITL instance by running `sim_vehicle.py` in the ArduCopter folder should result in Mission Planner automatically connecting. If not, check the baud rate is set to 115200 (at least that's what works for me). You can change the baud rate by right clicking the black region at the top, selecting Connection Options, then select UDP in the first drop down box and 115200 in the second drop down box.

## Connecting a Python Script to SITL
For a Python script to communicate with a drone (or proxy drone) using MAVlink, it must send MAVlink messages. These can be written directly into a script but can be cumbersome. Thankfully, the drone community has created a package called dronekit that makes writing drone commands much simpler. First check dronekit is installed in your environment and if not, run `pip install dronekit`. Dronekit also has its own way to launch SITL, so you can also run `pip install dronekit-sitl` if you think you'd like this.

Then, I followed [Caleb's videos](https://youtu.be/vkkwCGGbqTY?si=cdDX3c_ar5VFJ5T-) on using dronekit, expect I needed to add `--out=127.0.0.1:14550` when running `sim_vehicle.py` otherwise my SITL instance did not have an out port for my python script to connect to. The complete command I use when running an SITL instance is `sim_vehicle.py --map --console --out=127.0.0.1:14550`. The port 14550 is a default out port but you can also use 14551 and 14552. I believe these use the UDP protocol which I've seen is recommended. The other protocol TCP apparently can be used but I struggled to connect my scripts. It's default port is 5760.

With SITL running, I open a second Ubuntu CLI window. I then navigate to a folder outside of the ArduPilot directory Like Caleb, I've named this dk and this is where I keep my python scripts. Here, I have the following connection script, created by ChatGPT.
{% highlight python %}

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import builtins
import math
import argparse

######FUNCTIONS######

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()
	
	connection_string = args.connect
	
	if not connection_string:
			print("No connection string provided. SITL will be launched.")
			import dronekit_sitl
			sitl = dronekit_sitl.start_default()
			connection_string = sitl.connection_string()
	
	try:
		print(f"Connecting to vehicle on: {connection_string}")
		vehicle = connect(connection_string, wait_ready=True)
		print("Connected successfully!")
		return vehicle
	
	except socket.error as e:
		print(f"Socket error: {e}")
	except APIException as e:
			print(f"API exception: {e}")
	except Exception as e:
			print(f"Unexpected error: {e}")
		
	return None

######MAIN EXECUTABLE######

vehicle = connectMyCopter()

if vehicle:
	print("Vehicle is ready.")
else:
	print("Failed to connect to the vehicle.")

# Keep the script running
while True:
	time.sleep(1)  # Keeps the script running and maintains the connection

{% endhighlight %}


The script allows a user to parse a connection string, such as 127.0.0.1:14550, the port we chose when launching SITL in the first Ubuntu window. If not such connection string is parsed when launch this script, a new instance of SITL is launched. With `connection_string` defined, the script uses dronekit's `connect(connection_string)` function to connect to the drone. On running this script with `python connection_template.py --connect 127.0.0.1:14550` you should see the statement "Connected successfully!" printed into the second Ubuntu CLI window. You should also see some data printed in the first Ubuntu CLI window running the SITL, e.g. 
AP: Frame: QUAD/PLUS
Got COMMAND_ACK: REQUEST_AUTOPILOT_CAPABILITIES: ACCEPTED

We're now at a point we can write python scripts using dronekit that automates the behaviour of a drone (or proxy drone using SITL). Separate ground control software such as Mission Planner can connect to the proxy drone so we can see how the drone behaves as the script is run. In the next part of this project, we'll explore how to command a drone to fly a vertical parabolic manoeuvre that creates low gravity conditions. 