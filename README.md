# WinteROS_2025
## Forging the Mark-1 in the Cave  
<img src="Images/iron-man-iron-man-hammer.gif">

Welcome to **winteROS**.

Tony Stark was wounded - trapped in a cold, echoing cave, surrounded by enemies who demanded he build them a weapon.  
Instead, he found scraps of metal, broken sensors, old circuits… and a battered laptop running Linux.

On that laptop, one powerful framework was still installed:

**ROS2.**

To escape, Tony needed more than metal.  
He needed intelligence.  
He needed systems.  
He needed a machine that could *think*.

And this workshop is the guide that teaches him every skill required to build the first version of the **Mark-1 escape system**.

Every week strengthens a new subsystem.  
Every concept becomes part of the suit.  
Every step takes Tony closer to breaking out of the cave.

---

## ❄️ The Cave  
The walls rumble with distant machinery.  
Tony lies on a cot, sparks flickering from scavenged wires.  
He examines the scraps, the tools, the sensors.  
He knows one thing:

If he wants to survive, he must build something smarter than a weapon -  
he must build a machine capable of navigating, sensing, and acting on its own.

This is where winteROS begins:  
with knowledge that becomes machinery.

---

## ❄️ Week 0 — Powering Up the Cave Terminal (Installation Only)

Tony Stark wakes up in a cold cave with nothing but scrap metal, a half-broken laptop, and a car battery.
Before he can weld armor, power servos, or design intelligence, he needs one thing:

**A workstation that actually boots.**

This week is nothing fancy - just getting the cave terminal alive.

He installs Docker (or ROS2 natively), makes sure the system can start containers, and verifies that the environment responds. No nodes, no topics, no robots - just pure foundation.

By the end of Week 0, the dusty monitors flicker awake.
The cave hums with energy.
Tony finally has a machine he can build on.

---
<img src="Images/installationlol.jpg">

---

## ❄️ Week 1 - Learning and Building the Mark-1 Neural System

With the terminal working, Tony dives straight into ROS2 —  
beginning the construction of the first software systems that will run the Mark-1.

This week, he sets up a workspace, creates packages, and writes his first nodes.  
He works with topics, parameters, services, actions, launch files, and remaps —  
assembling every core component needed to control a robot from scratch.

He tests motion through turtlesim and inspects his system through `rqt`,  
watching each piece of StarkOS come alive on the screen.

By the end of the week, Tony has forged the **early brainstem** of the Mark-1:  
code that can communicate, react, coordinate, and respond.

The cave flickers brighter as terminals begin to glow.  
**StarkOS is waking up.**

---
<img src="Images/ros2.gif">

---

## ❄️ Week 2 - Forging the Body: The Mark-0 Cave Crawler

Tony now needs a physical platform - something that can survive the cave’s brutal tunnels and carry his systems through narrow passages.

Using Gazebo, he begins constructing a crude but powerful escape vehicle:

### **The Mark-0 Cave Crawler**  
A rugged, armored, four-wheeled prototype designed from leftover scrap.  
Not a suit yet - but a rolling testbed for everything the suit will become.

In simulation, Tony builds its chassis, joints, wheels, sensors, and physics.  
He mounts cameras and lidar onto the frame.  
He connects the motors to ROS2 topics to give the machine real movement.

Then he programs its first instinct:  
**avoid obstacles** using sensor data.

The vehicle crashes, reverses, finds new paths, and learns.  
Piece by piece, Tony refines it.

By the end of Week 2, the Cave Crawler becomes the Mark-1’s first metal backbone.

---

<img src=Images/robocar.png>

---

## ❄️ Week 3 - Vision & Autonomy: Teaching the Machine to See

To navigate the cave, Tony’s machine must see - truly see.

He integrates cameras and processes images through OpenCV.  
The system learns to detect shapes, markers, and objects.

Then, he enables SLAM - a technology that lets the Crawler **map the cave** while identifying its own position inside the maze of tunnels.

Navigation algorithms give the robot autonomy:  
it plans paths, avoids threats, and explores unknown areas without guidance.

This week, the Cave Crawler evolves from a vehicle into an intelligent explorer -  
the early mind of the Mark-1 suit.

---
<img src=Images/nav.png>

---

## ❄️ Week 4 - Manipulation: The Arm of the Mark-1

Movement and vision aren’t enough.

Tony needs the ability to **interact**: to lift debris, reach controls, manipulate objects, and eventually operate machinery during the escape.

He begins designing the mechanical arm - the prototype of what will one day become the iconic Iron Man gauntlet.

He studies kinematics, joint control, trajectories, and ROS2 interfaces for robotic arms.  
Then, he conceptualizes how this arm will mount onto his real robot.

This is where the machine becomes more than a vehicle -  
it becomes a tool for survival.

---
<img src=Images/roboarm.png>

---
## ❄️ Week 5 - Integration: Assembling the Mark-1 Escape System

The final step is integration.

Tony unites everything he has built:

- The Mark-0 Cave Crawler  
- Camera and sensor systems  
- SLAM and navigation logic  
- OpenCV-based vision  
- The robotic arm  
- The ROS2 control framework that powers it all  

He tests the complete sequence:

Explore.  
Map.  
Navigate.  
Detect obstacles.  
Use the arm to clear the exit path.

When the machine succeeds, Tony finally has what he needs:

**The first fully functional version of the Mark-1 escape system.**

The cave walls tremble as he begins the breakout.

---
<img src=Images/integrated.png>

---
## ❄️ Begin the Mission  
This is Tony Stark’s story -  
a story of building, learning, surviving.

Open the terminal.  
Power up ROS2.  
And help forge the Mark-1.

The escape begins now.

Head on to [Project Mark-1: Initialization Sequence](/Week%200/Project%20Mark-1%3A%20Initialization%20Sequence.md) and power up the cave terminal.

> Note: If you already have ROS2 Jazzy installed and initialized to launch on your system then skip the `Initialization Sequence` and head on to [Week 0](/Week%200/week0.md).
