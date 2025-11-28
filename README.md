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
### Raspberry Pi Dependencies
```bash
sudo apt install python3-full python3-venv
python3 -m venv depthai-venv
source depthai-venv/bin/activate
apt-get update && upgrade
curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
python3 -m pip install depthai
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python
cd examples
sudo python3 install_requirements.py
cd
pip3 install opencv-python
pip3 install -U numpy
python3 -m pip install depthai
cd depthai-python/examples
```
Test with a preview before continuing.
```bash
python3 ColorCamera/rgb_preview.py
```
If the camera works at this stage, continue with the installation. For refinement the efficacy of dependencies after this point should be reviewed. 
```bash
git clone https://github.com/luxonis/depthai.git
cd depthai
python3 install_requirements.py
cd
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

source depthai-venv/bin/activate
chmod +x myscript.py
./myscript.py

