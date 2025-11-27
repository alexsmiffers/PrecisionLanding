# PrecisionLanding
Precision Landing System using Raspberry Pi Zero and OAK-D-LITE-AF Camera

## Raspberry Pi
### Raspberry Pi VNC
For testing it is best to use VNC to see the GUI of the PI.

First:
1. Power on the Pi
2. Connect to same network
3. SSH in
    ssh <username>@<ip>
4. sudo raspi-config
5. Interface Options > VNC > Enable > Finish
6. Check VNC is active using:
    'systemctl status wayvnc'
7. Join on viewer device using a VNC client like TigerVNC. 
