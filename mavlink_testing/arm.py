from pymavlink import mavutil

# Start a connection listening to a UDP port
m = mavutil.mavlink_connection('tcp:192.168.10.233:5762')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
m.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (m.target_system, m.target_component))

m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
msg = m.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)