---
title: Micro Drop Drone 4 - Lua Scripting
date: 2025-04-03
categories:
  - micro drop drone
tags:
  - drones
  - low-gravity
  - projects/micro_drop_drone
---

# Update
For the past few months, I have been mostly focused on other tasks. During the time I spent on this project, I explored writing Python scripts that create low gravity conditions. My most recent Python script could get the drone's acceleration down to around 0 m/s<sup>2</sup> but only for a few hundredths of a second. The thrust would increase quickly, suggesting there was an issue with the logic in the script. I may continue with the Python script approach in the future.

But this approach has an elephant in the room. The Python script runs on the ground control stations (GCS), and not on the drone. The script uses telemetry transmitted by the drone and transmits MAVLink commands to control the drone all over the radio link . This naturally adds some delay reducing responsiveness, and if the radio communication is patchy or drops out, the drone's low gravity manoeuvre stops. 

To increase performance and reliability, the script should run on the drone itself. There are two ways to do this:
1. Add a companion computer (such as a Raspberry Pi) that runs the script and sends MAVLink commands via a physical connection to the FC.
2. Run the script on the FC.

Option 1 adds more cost and complexity so will only be explored if necessary. Option 2 is simpler. Flight controllers such as the Pixhawk have some flash memory to run scripts, but it's limited (2-4 MB). The ArduPilot community have implemented Lua scripting methods that allow users to run scripts directly on their FC. Lua was chosen since its lighter compared to similar Python scripts, meeting the constraints of the FC's flash. 

# Lua Scripts
[Lua](https://www.lua.org/) is an... interesting language to use. For scripts to run in conjunction with ArduPilot, Lua is useful since its pretty readable (feels similar to Python in that sense) and light enough to run on a Pixhawk.

The ArduPilot community have created their own Lua methods to communicate with ArduPilot FC software, so writing scripts for the average user is much easier. [The ArduPilot page for Lua scripts](https://ardupilot.org/copter/docs/common-lua-scripts.html) is a good place to start but can be outdated. It's best to check the available methods in the [Lua doc](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/docs/docs.lua) in ArduPilot's repo. I'll refer to this as the golden list.

## Useful Methods
I first needed to know what methods were useful for my application. I went through the golden list, picked out interesting methods, and tested them in isolation in Mission Planner (see next section). The following methods are useful:
- `baro:get_altitude()` <- get the altitude relative to the initial altitude (when the drone was powered on)
- `ahrs:get_location()` <- returns a location object that includes altitude data
- `ahrs:get_velocity_NED()` <- get the velocity vector of the drone in the NED frame
- `ins:get_accel(0)` <- get the acceleration of the drone from the IMU (note this is in the drone's body frame)
- `motors:get_throttle()` <- get the throttle (0-1) of the drone's motors
- `arming:is_armed()` <- check if the drone is armed
- `battery:capacity_remaining_pct(0)` <- check the battery percentage of the drone
- `gcs:send_text(0, "LGM: " .. message)` <- send a message to the ground control station (with the prefix LGM:)

## Testing Lua Scripts with Mission Planner
Mission Planner is a ground control station software that is able to run SITL simulations like that described in my [last post](/_posts/2024-12-16-micro-drop-drone-3.md). Go to the Simulation tab and select Multirotor. You will need to download the data the first time but can select Skip Download in the drop down box for future runs of the SITL.
![](/assets/images/mdd_blog_post_4/mdd_MP_1.png "Starting an SITL simulation using Mission Planner.")
Once running, navigate to Config, MAVFtp, click the + to open the folder, then double click scripts. If this scripts folder doesn't exist, you will need to create it. Here, you can add your Lua scripts by right clicking and select upload.
![](/assets/images/mdd_blog_post_4/mdd_MP_2.png "Uploading Lua scripts to your SITL instance.")
Then you need to restart the SITL for the drone to boot with the loaded Lua script. A quick way to do this is press `ctrl + F`, click reboot Pixhawk, wait a few seconds until the drone disconnects, then click connect in the top right corner. This doesn't always work, so you may need to just disconnect and restart the SITL like at the beginning. When the SITL drone starts up, the script will be active.


# LGM v1
With the help of Anthropic's Claude, I put together a v1 of a Low Gravity Manoeuvre script written in Lua. This script, `lgm_v1.lua`, is structured as a state machine. The drone starts of in an IDLE state and only transitions to other states when conditions have been met (such as flight checks). This structure allows easy implementation of parallel states such as an abort state if, say, the drone is too low or or moving too fast.

{% highlight lua %}
-- Low Gravity Maneuver (LGM) Script for Pixhawk Flight Controller - written by Claude
-- Definitions:
-- - Low gravity: accelerations < 0.1 g's = 0.981 m/s²
-- - GCS: ground control station
-- - acc_z: vertical acceleration where upwards is positive
-- - FC: flight controller

--@diagnostic disable: cast-local-type

-- Configuration parameters
local MIN_ALT_START = 2.0            -- Minimum altitude to start maneuver (meters)
local MIN_ALT_DURING = 10.0          -- Minimum safe altitude during low gravity phase (meters)
local MIN_BATTERY_START = 80         -- Minimum battery percentage to start (%)
local MIN_BATTERY_DURING = 20        -- Minimum battery percentage during low gravity phase (%)
local MAX_VELOCITY = 0.01            -- Maximum velocity considered as "stopped" (m/s)
local MAX_UP_VEL = 5                 -- Maximum velocity during the ascent (m/s)
local DATA_SAMPLING_RATE = 20        -- Hz for data recording
local LOW_G_DURATION = 1.5           -- Duration of low gravity phase (seconds)
local TARGET_ACC_Z = 0.09            -- Target maximum vertical acceleration during low gravity phase (g's)
local AUX_CHANNEL = 7
local THROTTLE_RC = 3                -- RC channel for throttle
local HOVER_THRUST = 0.55            -- Value of (actual) throttle for SITL drone to hover 
local AuxSwitchPos = {LOW=0, MIDDLE=1, HIGH=2}

-- Get throttle channel parameters
local throttle_min = param:get(('RC%d_MIN'):format(THROTTLE_RC))
local throttle_max = param:get(('RC%d_MAX'):format(THROTTLE_RC))
local throttle_ch = rc:get_channel(THROTTLE_RC)

-- State machine states
local STATES = {
    IDLE = 0,
    PREFLIGHT_CHECK = 1,
    ASCENDING = 2,
    LOW_GRAVITY = 3,
    DESCENDING = 4,
    ABORT = 5
}

-- Global variables
local current_state = STATES.IDLE
local maneuver_start_time = 0
local low_g_start_time = 0
local data_log = {}
local last_sample_time = 0
local calibrated_low_g_thrust = nil


--------- Useful Functions -----------

-- Function to rotate a vector from the body frame of a drone to the local NED frame --
function rotate_to_NED(ax, ay, az, pitch, yaw, roll)
    local cos = math.cos
    local sin = math.sin

    --[[ Convert angles to radians
    local phi = math.rad(roll)
    local theta = math.rad(pitch)
    local psi = math.rad(yaw)--]]

    local phi = roll
    local theta = pitch
    local psi = yaw

    -- Corrected Rotation Matrix (Body to NED)
    local R = {
        {cos(theta) * cos(psi), sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)},
        {cos(theta) * sin(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)},
        {-sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta)}
    }

    -- Rotate the acceleration vector
    local ax_ned = R[1][1] * ax + R[1][2] * ay + R[1][3] * az
    local ay_ned = R[2][1] * ax + R[2][2] * ay + R[2][3] * az
    local az_ned = R[3][1] * ax + R[3][2] * ay + R[3][3] * az

    return {ax_ned, ay_ned, az_ned}
end



--------- Drone Specific Functions ---------

-- Function to check if the AUX switch is activated
function aux_switch_is_on()
    --below is for AUX_CHANNEL on RC
    --return rc:get_pwm(AUX_CHANNEL) > 1500

    return rc:get_aux_cached(300) == AuxSwitchPos.HIGH -- 300 for Scripting 1 (must also set for RCX_OPTION in MP - I chose RC14)
end

-- Function to check if the drone is armed
function is_armed()
    return arming:is_armed()
end

-- Function to get current altitude
function get_altitude()
    return baro:get_altitude() -- uses barometer to measure altitude since boot up
    --[[
    Add the AHRS:get_locaton method here and compare both values?
    --]]
end

-- Function to get current vertical velocity
function get_vertical_velocity()
    local vel = ahrs:get_velocity_NED()
    if vel then
        local vz = -vel:z() -- change sign so up is +ve
        return vz
    end
end

-- Function to get current vertical acceleration
function get_vertical_acceleration()
    -- Get IMU acceleration and orientation
    local acc = ins:get_accel(0)
    local pitch = ahrs:get_pitch()
    local yaw = ahrs:get_yaw()
    local roll = ahrs:get_roll()

    if acc and pitch and yaw and roll then
        -- Rotate acceleration vector acc from body frame to NED frame
        local acc_NED = rotate_to_NED(acc:x(), acc:y(), acc:z(), pitch, yaw, roll)

        -- Extract z component of drone's acceleration in NED frame
        local acc_z_NED = acc_NED[3]
        return acc_z_NED
    end
end

-- Function to get battery percentage
function get_battery_percentage()
    return battery:capacity_remaining_pct(0) -- battery on instance = 0
end


-- Function to send status message to GCS
function send_status(message)
    gcs:send_text(0, "LGM: " .. message)
end


-- Function to convert throttle percentage to PWM value
local function get_throttle_pwm(throttle_percent)
    -- Ensure throttle_percent is between 0 and 1 (no reverse for quadcopter)
    throttle_percent = math.max(0, math.min(1, throttle_percent))
    
    -- Convert percentage to PWM value (linear mapping from min to max)
    return math.floor(throttle_min + ((throttle_max - throttle_min) * throttle_percent)) --using floor to remove decimals since overide() only takes integers
end


-- Function to set the throttle
function set_throttle(target_throttle)
    -- Calculate PWM value from target throttle
    local throttle_pwm = get_throttle_pwm(target_throttle)
    --send_status(throttle_pwm) --<<-- for debugging
    -- Set the throttle override
    --throttle_ch:set_override(throttle_pwm)

    -- New Method --
    vehicle:set_target_rate_and_throttle(0, 0, 0, target_throttle)
end


-- Flight check function
function flight_check()
    if not is_armed() then
        send_status("FAILED: Vehicle not armed")
        return false
    end
    
    if get_altitude() < MIN_ALT_START then
        send_status("FAILED: Altitude too low (< " .. MIN_ALT_START .. "m)")
        return false
    end
    
    if get_vertical_velocity() > MAX_VELOCITY then
        send_status("FAILED: Vehicle not stationary")
        return false
    end
    
    if get_battery_percentage() < MIN_BATTERY_START then
        send_status("FAILED: Battery too low (< " .. MIN_BATTERY_START .. "%)")
        return false
    end
    
    -- All checks passed
    send_status("All checks PASSED. Starting maneuver.")
    return true
end

-- Function to calibrate the thrust needed for low gravity --<<-- need to add control system here
function calibrate_lgm()
    -- This would typically involve a lookup table or calculation based on
    -- current vehicle weight, battery level, etc.
    -- For now, using a simplified estimate (50% of hover thrust)
    return HOVER_THRUST * 0.5
end

-- Function to perform safety checks during maneuver
function perform_safety_checks()
    -- Check if we should abort the maneuver
    if current_state ~= STATES.IDLE and current_state ~= STATES.ABORT then
        -- Check altitude
        if get_altitude() < MIN_ALT_DURING then
            send_status("ABORT: Altitude too low")
            abort_sequence()
            return false
        end
        
        -- Check battery
        if get_battery_percentage() < MIN_BATTERY_DURING then
            send_status("ABORT: Battery too low")
            abort_sequence()
            return false
        end
        
        -- Check timeout (prevent getting stuck in a state)
        local current_time = millis()
        if current_state == STATES.ASCENDING and current_time - maneuver_start_time > 5000 then
            -- If ascending for more than 5 seconds
            send_status("ABORT: Ascent timeout")
            abort_sequence()
            return false
        elseif current_state == STATES.LOW_GRAVITY and current_time - low_g_start_time > LOW_G_DURATION * 1000 + 2000 then
            -- If in low-g for more than expected time + 2 seconds
            send_status("Low-g phase complete, descending")
            current_state = STATES.DESCENDING
            return true
        end
    end
    
    return true
end

-- Function to start ascent phase
function start_ascent()
    maneuver_start_time = millis()
    send_status("Starting ascent phase")
    
    -- set mode to GUIDED for vehicle object methods
    vehicle:set_mode(4)

    -- Set to maximum thrust for ascent
    set_throttle(1.0)
    --send_status(string.format("act thr:%f gcs thr:%f", motors:get_throttle(), gcs:get_hud_throttle()))
end

-- Function to check if maximum vertical velocity is reached
function check_max_velocity()
    return get_vertical_velocity() > MAX_UP_VEL
end

-- Function to start low gravity phase
function start_low_gravity()
    low_g_start_time = millis()
    send_status("Starting low gravity phase")
    
    -- Calibrate thrust for low gravity if not already done
    if calibrated_low_g_thrust == nil then
        calibrated_low_g_thrust = calibrate_lgm()
    end
    --send_status(string.format("actual throttle lgm:%f", motors:get_throttle()))
    -- Clear previous data
    data_log = {}
    last_sample_time = millis()
end

-- Function to maintain low gravity conditions
function maintain_low_gravity()
    -- Apply calibrated thrust
    set_throttle(calibrated_low_g_thrust)
    
    -- Record data at specified sampling rate
    local current_time = millis()
    if current_time - last_sample_time >= (1000 / DATA_SAMPLING_RATE) then
        record_data()
        last_sample_time = current_time
        
        -- Simple PID-like adjustment for thrust based on measured acceleration
        local acc_z = get_vertical_acceleration()
        if acc_z < TARGET_ACC_Z * 9.81 + 0.05 then --<- converts target acc from g's to m/s^2 (also only works for upward half)
            -- Too much acceleration, reduce thrust
            calibrated_low_g_thrust = calibrated_low_g_thrust * 0.98
        elseif acc_z > TARGET_ACC_Z * 9.81 - 0.05 then --<- converts target acc from g's to m/s^2
            -- Too little acceleration, increase thrust
            calibrated_low_g_thrust = calibrated_low_g_thrust * 1.02
        end
    end
end

-- Function to start descent/deceleration phase
function start_descent()
    send_status("Starting controlled descent")
    -- Apply deceleration thrust (more than hover to slow down)
    set_throttle(HOVER_THRUST * 1.5)
end

-- Function to check if vehicle has stopped
function check_stopped()
    return get_vertical_velocity() < MAX_VELOCITY
end

-- Function to record acceleration data
function record_data()
    local timestamp = millis() - maneuver_start_time
    local acc_z = get_vertical_acceleration()
    local alt = get_altitude()
    local vel_z = get_vertical_velocity()
    --local throttle = motors:get_throttle()
    
    table.insert(data_log, {
        time = timestamp,
        acc_z = acc_z,
        altitude = alt,
        vel_z = vel_z
        --throttle = throttle
    })
end

-- Function to save recorded data
function save_data()
    -- Create CSV file
    local filename = string.format("/logs/lgm_%s.csv", tostring(millis()))
    local file = io.open(filename, "w")
    
    if file then
        -- Write header
        file:write("time_ms,acc_z,altitude,vel_z\n")
        --file:write("time_ms,acc_z,altitude,vel_z,throttle\n")
        
        -- Write data
        for _, data in ipairs(data_log) do
            file:write(string.format("%s,%.4f,%.4f,%.4f\n", tostring(data.time), data.acc_z, data.altitude, data.vel_z))
            --file:write(string.format("%s,%.4f,%.4f,%.4f,%d\n", tostring(data.time), data.acc_z, data.altitude, data.vel_z, data.throttle))
        end
        
        file:close()
        send_status("Data saved to " .. filename)
    else
        send_status("Failed to save data")
    end
end

-- Function to handle emergency abort
function abort_sequence()
    current_state = STATES.ABORT
    send_status("ABORTING MANEUVER")
    
    -- Apply deceleration thrust to stop safely
    set_throttle(0.8)
    
    -- After a safe stop, return to IDLE state
    if check_stopped() then
        current_state = STATES.IDLE
        send_status("Abort complete, vehicle stopped")
        save_data()
    end
end

-- Main update function (called at regular intervals by Pixhawk)
function update()
    if aux_switch_is_on() then

        if current_state == STATES.IDLE then
            if flight_check() then
                current_state = STATES.PREFLIGHT_CHECK
            end
        elseif current_state == STATES.PREFLIGHT_CHECK then
            current_state = STATES.ASCENDING
            start_ascent()
        elseif current_state == STATES.ASCENDING then
            if check_max_velocity() then
                current_state = STATES.LOW_GRAVITY
                start_low_gravity()
            end
        elseif current_state == STATES.LOW_GRAVITY then
            local current_time = millis()
            maintain_low_gravity()
            
            -- Check if low gravity phase is complete
            if current_time - low_g_start_time >= LOW_G_DURATION * 1000 then
                current_state = STATES.DESCENDING
                start_descent()
            end
        elseif current_state == STATES.DESCENDING then
            if check_stopped() then
                current_state = STATES.IDLE
                send_status("Maneuver complete")
                save_data()
            end
        elseif current_state == STATES.ABORT then
            -- Continue abort sequence until stopped
            if check_stopped() then
                current_state = STATES.IDLE
                send_status("Abort complete, vehicle stopped")
                save_data()
            end
        end
    else
        -- AUX switch turned off
        if current_state ~= STATES.IDLE then
            abort_sequence()
        end
    end
    
    -- Always perform safety checks
    perform_safety_checks()

    return update, 100 -- Run at 10Hz
end

send_status("Script Ready")
return update, 1000 -- start script after one second
{% endhighlight %}


This version has the foundation for an lgm script, but its control algorithm to achieve low gravity is very simple. It starts with a low thrust during the low gravity phase and increases the thrust by a small amount if the acceleration is too low and decreases the thrust if the acceleration is too high.

The graph below shows the vertical acceleration, velocity and altitude of a proxy drone during the low gravity phase when controlled by this lgm_v1.lua.
![](/assets/images/mdd_blog_post_4/flight_data_plot_v1_data_2.png "Flight data of a proxy drone being controlled by lgm_v1.lua.")
The acceleration hovers around 0.5 g's, which, while lower than 1 g, is not considered low gravity (typically below 0.1 g's). The next steps of this project is to implement more effective control algorithms that achieve low gravity conditions. In my next post, I'll also explain more about the design of these lgm scripts.