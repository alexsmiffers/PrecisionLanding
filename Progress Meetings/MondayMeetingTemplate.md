# Weekly Meeting Minutes — Start of Week

## Date and Time

01/12/2025
13:20

---

## Current Progress Update

* Summary of work completed since last meeting:

  * Developed script main.py which can be configured to get the pos of aruco or charuco.
  * The monocolour stereo cameras can use a greater framerate and have been getting better detection.
  * Tested the pose estimation and it is accurate to approx 1cm.
  * 

---

## Deliverables for the Week

* Deliverable 1:

  * 
  * Due: Friday 12th December
  
* Deliverable 2:

  * 
  * Due: Friday 12th December

* Deliverable 3:

  * 
  * Due: Friday 12th December
---

## Goals and Priorities

* Primary goals for this week:

  * 
  * 
  * 

---

## Risks / Blockers

* Any known issues that may impact progress:

  * Danger of poorly tested code in a real flight configuration.
  * Limited access to gcs and linux for ros simulation.

---

## Stretch Goals and Project Goals

* Stretch goals for the project in order of priority.

  * Functional landing system using Raspi Zero and Pixhawk.
  * Refine code and parameters for better performance.
  * Try different camera types and resolutions to evaluate best camera choice.
  * Translate code from Python3 to C++ and evaluate performance changes.
  * Analyze computer resource requirements and choose appropriate lightweight microcomputer for this project.
  * Develop PCB for new microcomputer.
  * Develop lightweight housing and assembly for the microcomputer and camera.

---

## Notes / Additional Comments

* Need to check the purpose of the guided mode.
* need to 
* robustness and detection range
* larger size means reduced detection range, 4x4 is better range
* robustness is reduction of false readings, 6x6 is better for robustness
* 5x5 might be sweetspot
* no need for custom dict
* start with 50 for size
at 15m can only detect largest
as you get closer like 1m large marker will be out of the frame
check with SITL first
use 4 tech for the reading

use sq pnp for pose estimation
sitl already cofnigured on the pi

