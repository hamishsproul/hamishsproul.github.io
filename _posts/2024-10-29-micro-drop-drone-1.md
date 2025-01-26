---
#layout: posts
title: Micro Drop Drone 1 - Introduction
date: 2024-10-29 10:00:00 +0530
categories:
  - micro drop drone
tags:
  - drones
  - low-gravity
---

# Backstory

Early on in my Space Studies degree, I became interested in low and microgravity platforms. I was surprised by the cost to use platforms such as drop towers and parabolic flights, and that there are so few offerings. It seemed for space technology developers to test their systems in microgravity, they would either need to wait ~12-18 months for an offering to become available or take the risk and send a prototype into space! With the increasing problem of space debris, IOD is not a sustainable approach. As many have pointed out, there is a need for a more accessible microgravity platform; one which developers can access within days or even own themselves; something like a wind tunnel but for microgravity.

So, I focused my master thesis on this topic. I defined customer requirements based on some ZBLAN and SiC experiments performed on parabolic flights. I translated these into design requirements and modelled the flight dynamics of a microgravity platform that meets these requirements. To keep things simple, the platform would (ideally) travel vertically, like a drop tower. Horizontal motion is not needed to create microgravity conditions so it's best to remove this and save the action. I then performed a trade study for different design options. The upshot was a ~2 m tall capsule powered by 4 micro jet engines with reverse thrust capability. At each end of the capsule are stabilising fins with embedded planar electric propellers for attitude control during a flight. My models suggest this could place a 15 kg payload in low gravity for around 25 s.

# Project Aim

Modelling the performance of a drop drone was fun, however, it was all theoretical. The real fun is in bringing something to life! In my thesis, I estimated the production costs of a drop drone to be EUR 250,000, not a sum I have lying around… A much smaller version using COTS drone parts is more affordable. A so-called micro drop drone will only achieve 3-5 s of low gravity carrying payloads < 1 kg. Such performance is unlikely to be useful for industry but could make for an enaging educational activity for students.

My aim, therefore, is to build and program a micro drop drone that performs a vertical parabolic manoeuvre to create low gravity conditions. In these posts, I’ll document my progress so hopefully you can create your own micro drop drone and start performing low gravity exepriments!
