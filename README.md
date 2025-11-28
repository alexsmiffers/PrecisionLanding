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
# Installs library and requirements
python3 examples/python/install_requirements.py
``` 

### Test Raspberry Pi
``` bash
cd examples/python
# Run YoloV6 detection example
python3 DetectionNetwork/detection_network.py
# Display all camera streams
python3 Camera/camera_all.py
``` 

source depthai-venv/bin/activate
chmod +x myscript.py
./myscript.py
