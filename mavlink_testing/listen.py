from pymavlink import mavutil

# Start a connection listening to a UDP port
m = mavutil.mavlink_connection('tcp:192.168.10.233:5762')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
m.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (m.target_system, m.target_component))

while 1:
    msg = m.recv_match(type='ATTITUDE', blocking=True)
    print(msg)