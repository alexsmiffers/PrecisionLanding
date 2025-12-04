# PrecisionLanding
Precision Landing System using Raspberry Pi Zero and OAK-D-LITE-AF Camera

## Raspberry Pi
### Raspberry Pi VNC
For testing it is best to use VNC to see the GUI of the PI.

First:
1. Power on the Pi
2. Connect to same network
3. SSH in
    ```bash
    ssh <username>@<ip>
    ```
4. Open Raspberry Pi Config
    ```bash
   sudo raspi-config
    ```
6. Interface Options > VNC > Enable > Finish
7. Check VNC is active using:
   ```bash
    systemctl status wayvnc
   ```
8. Join on viewer device using a VNC client like TigerVNC. 

### Raspberry Pi Dependencies v2
``` bash
git clone https://github.com/luxonis/depthai-core.git && cd depthai-core
python3 -m venv venv
source venv/bin/activate
pip3 install MAVProxy
pip install future
pip3 install pymavlink
pip install pyyaml
pip install pyserial
# Installs library and requirements
cd depthai-core
python3 examples/python/install_requirements.py
cd ..
git clone https://github.com/opencv/opencv_contrib.git
``` 

### Test Raspberry Pi
``` bash
cd examples/python
# Run YoloV6 detection example
python3 DetectionNetwork/detection_network.py
# Display all camera streams
python3 Camera/camera_all.py
``` 
### Start MavLink on Raspberry Pi
```bash
source venv/bin/activate
mavproxy.py --master=/dev/serial0,57600
```

source depthai-venv/bin/activate
chmod +x myscript.py
./myscript.py

mavproxy.py --master=/dev/serial0,57600

OpenCV / ArUco camera frame (typical)
X: right, Y: down, Z: forward (optical axis).

Great Resource:
https://landmarklanding.com/blogs/landmark-lab-notes/ardupilot-precision-landing?srsltid=AfmBOooBo9DkUGmmoWkkiA64ibtNPDiqPz8BiBHSH0zS63usgCl28VDP

Citation for OpenCV:
‍S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez. 2014. "Automatic generation and detection of highly reliable fiducial markers under occlusion". Pattern Recogn. 47, 6 (June 2014), 2280-2292. DOI=10.1016/j.patcog.2014.01.005

# Notes:
* Once flight controller (FC) is finalised investigate powering raspi directly from FC to minimise wiring. Only works on boards which TELEM outputs +5V not +3V3.
* Global shutter is ideal but not required, evaluate the difference in some options.
* Suggests raspi zero 2 W or better which has 1.1 GHz quad core processor and 512 MB ram.
* Future stretch goals could include using IR sensors for better performance in low light conditions like in this research: https://arxiv.org/abs/2403.03806
* can use yaml or argparser, not sure which is better for this application yet, probs settings as it doesn't involve editing shell script
* pose estimation starts failing at approx 15 cm from the board.
* currently max time for a loop is 0.08 using python on macbook. Raspi not yet evaluated. This would require a hz of approx 10 which is the minimum recommended.


