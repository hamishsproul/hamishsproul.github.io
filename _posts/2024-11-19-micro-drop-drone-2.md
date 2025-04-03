---
title: Micro Drop Drone 2 - The Hardware
date: 2024-11-19 13:00:00 +0530
categories:
  - micro drop drone
tags:
  - drones
  - low-gravity
  - projects/micro_drop_drone
---

The control algorithm will be the key technology in a micro drop drone. There are two options to test control algorithms: using software-in-the-loop (SITL) applications or using a prototype drone. To enable this second option, I started by building the base platform of a micro drop drone prototype.

## Design Requirements
The following requirements will ensure a micro drop drone can meet the needs of students exploring 3-5 seconds of low gravity.

To maximise accessibility, a micro drop drone will use a widely available drone platform. 
1. The micro drop drone will have a high thrust to weight ratio > 1.7 to maximise its initial velocity during the low gravity phase. 
2. The micro drop drone will be able to carry a payload > 250 g.
3. The micro drop drone will have a payload volume > 1000 cm<sup>3</sup>.
4. The micro drop drone will weigh < 2500 g.
5. The micro drop drone will fly below 120 m.

## The Approach
A micro drop drone will need to fly a pre-programmed flight profile. There are several COTS drones that allow users to program their behaviour, such as the [RoboMaster TT](https://www.dji.com/robomaster-tt?site=brandsite&from=landing_page) or the [CoDrone Mini](https://www.robolink.com/products/codrone-mini). But, since they are small, these have limited performance, and their hardware is not easily customizable.

ArduPilot flight controllers such as the range of Pixhawks can be automated with ground control station software or with scripts. They can also be connected to any suitable hardware. Due to this flexibility, I decided to use a Pixhawk flight controller.

### Design Choice
There are many drone kits available using Pixhawk flight controllers. You can also buy components individually and build your own from scratch. I decided to buy [this drone kit](https://www.aliexpress.com/item/1005004806035219.html?gatewayAdapt=4itemAdapt) using the common DJI F450 frame (450 mm diagonal). I opted with the 915 MHz radio telemetry package since this frequency suits the US and UK well and the 500 MW variant for extra range. The kit contains the following:
  * a Pixhawk 2.4.8
  * four 2212 motors - these size motors are common for F450 sizes and are rated at around 920 kV (this means they rotate at 920 RPM or 15.3 times a second when 1 volt is applied)
  * four 1045 propellers (each prop blade is 10.45 cm long)
  * four BLHELI 30A electronic speed controllers
  * a 915MHZ 500MW Radio telemetry system - good range and useable in the US and UK
  * an M8N GPS
  * a 3DR power module
Extra peripherals included are an OLED screen, buzzer, safety switch, USB type-C with RGB LED, and an I2C expansion board to connect these peripherals. The kit does not include a battery or a receiver to operate the drone manually.

On top of this kit, I bought [these 4S batteries](https://www.aliexpress.com/item/1005007439526090.html?gatewayAdapt=4itemAdapt) with [this LiPo balance charger](https://www.aliexpress.com/item/1005004771454637.html?gatewayAdapt=4itemAdapt), and [this FlySky FS-i6X transmitter](https://www.aliexpress.com/item/1005006860791905.html?gatewayAdapt=4itemAdapt) with an IA10B receiver. For safe charging and storage, I also bought [this LiPo safety bag](https://www.aliexpress.com/item/1005004478094343.html?gatewayAdapt=4itemAdapt).

### Costs
I bought these products from different vendors on AliExpress during late October 2024. The total came to £291.57. The majority of this cost was the drone kit at £189.63.

### Potential Performance
The thrust from each motor depends on its angular speed which depends on the applied voltage. For a full 4S LiPo battery, the supplied voltage is 16.8V. Each motor can spin at a maximum 16.8 V ⨉ 15.3 rev/V s = 257 rev/s. Assuming a propellor diameter of 21 cm and a thrust coefficient of 0.1 (lower end of typical range), each propellor can provide a maximum thrust of 15.8 N. The total maximum thrust of the drone is then 63.2 N. Taking the upper mass limit as 2.5 kg, the drone has a maximum thrust to weight ratio (TWR) of 2.57, well above the 1.5 - 1.7 I was modelling in my thesis. Taking the lower limit of the battery voltage, 14.8 V, the maximum thrust of the drone is 49.2 N, giving a TWR of  2.

![](/assets/images/mdd_blog_post_2/mdd_solve_ivp_1.png "A Model a Micro Drop Drone's Performance with a TWR of 2")

Using my model from my thesis, a 2.5 kg drone with a TWR of 2 flying a vertical parabolic trajectory under 116 m can achieve a 5.8 seconds of low gravity end 42 m above the ground for a safe descent. Notice the power spikes above 1 kW. This might be an issue. At 14.8 V (low battery), the motors spin at 227 rev/s, resulting in a mechanical power estimate of 176 W per motor. A total 704 W is too low and limits the duration of low gravity to less than 3 s. But at full battery, each motor can spin at 256 rev/s, resulting in 256 W per motor, enough for more than 5 s of low gravity. 

## The Build
On day one, I soldered the ECS’s and XT60 power cable to the power distribution board (the large board that makes up the centre of the drone frame). I then attached each motor to a leg and attached these to the power distribution board with the supplied screws. I secured these legs by connecting the smaller board on top. 


| 1 | 2 | 3 |
:-------------------------:|:-------------------------:|:-------------------------:
![](/assets/images/mdd_blog_post_2/PXL_20241021_171654524.jpg)   | ![](/assets/images/mdd_blog_post_2/PXL_20241021_174116037.jpg) | ![](/assets/images/mdd_blog_post_2/PXL_20241021_180451121.jpg)



One day two, I secured the Pixhawk 2.4.8 to the damping mount and glued this to the top board of the drone. I then secured the switch, buzzer, power module, radio telemetry module, OLED display, I2C extension board, and GPS mount to the drone. I connected everything up following [this ArduPilot guide](https://ardupilot.org/copter/docs/autopilot-assembly-instructions.html). The following photos show the placement of each component.

| 1 | 2 |
:-------------------------:|:-------------------------:
![](/assets/images/mdd_blog_post_2/PXL_20241119_165359072.jpg)   | ![](/assets/images/mdd_blog_post_2/PXL_20241119_165443471.jpg)


The mass of the drone without the battery and propellers is 827 g and with a 4S battery and four propellers is 1222 g.

Once built, I loaded PX4 firmware onto the drone as I thought this would allow for smoother automation with Python scripts later on. However, I ran into some issues and struggled to get the drone to arm. I changed the firmware to ChibiOS, the firmware recommended by the drone kit seller. This proved to be a lot smoother and within a few hours of playing around, I was able to arm my drone and get several GPS satellite connections. 

### First Flight
During this build, I have been based in Washington DC, a no-fly zone for recreational drone operators. I had to wait until I travelled out into Virginia to test fly my drone. After passing the Recreational Drone Safety Test (TRUST), registering my drone with the FAA, and attached a Remote ID module to the drone, I was ready for my first test.

The first flight occurred on 10/11/2024. The following video shows one test from this flight. 

{% include youtube.html id="dfwy-Sw0jS8" %}


While my flying skills can be improved, the drone operation was smooth and without issues. The drone took off well and was as stable as other drones I’ve flown. 

While python scripts can be loaded onto a flight controller like a Pixhawk, flight controllers only have so much computing power. As I develop the software for this project, I will look at purchasing a companion computer that runs the script (probably a Raspberry Pi).


