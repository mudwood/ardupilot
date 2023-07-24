from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Vehicle
from pymavlink import mavutil
import time
import mpreader

##### func

##### body

# vQplane = connect('udp:127.0.0.1:14561', wait_ready=True, timeout=60 )      # quadplane
# vCopter = connect('udp:127.0.0.1:14571', wait_ready=True, timeout=60 )      # copter
# vBoat   = connect('udp:127.0.0.1:14581', wait_ready=True, timeout=60 )      # boat
# vRoverR = connect('udp:127.0.0.1:14591', wait_ready=True, timeout=60 )      # rover rinsetsu
vRoberT = connect('udp:127.0.0.1:14601', wait_ready=True, timeout=60 )      # rover taigan

print("Connected.")

vRoberT.parameters[ 'WP_RADIUS' ] = 5   # wpマージン(m)

# ##### ミッションの内容（一度DLして cmds を初期化しないとうまくいかない？）
cmds = vRoberT.commands
cmds.download()
cmds.wait_ready()

# cmd22 = MAV_CMD_NAV_TAKEOFF
# cmd16 = MAV_CMD_NAV_WAYPOINT
# cmd21 = MAV_CMD_NAV_LAND

fileName = '/home/ardupilot/ardupilot/dev-app/day5/rover_Taigan2St.txt'
( commands, lats, lons, alts ) = mpreader.mpReader( fileName )

for command, lat, lon, alt in zip( commands, lats, lons, alts ):
    print( command, lat, lon, alt )

cmds = vRoberT.commands
cmds.clear()                                            # 現在のミッションをクリア
cmds.upload()
cmds.wait_ready()

for command, lat, lon, alt in zip( commands, lats, lons, alts ):
    if command == 22:       # MAV_CMD_NAV_TAKEOFF
        cmd = Command(  0,                                           # system id
                        0,                                           # component id
                        1,                                           # seq
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   #frame
                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    # MAV_CMD
                        0,   # current
                        1,   # auto continue
                        0,   # param 1
                        0,   # param 2
                        0,   # param 3
                        0,   # param 4
                        lat,   # param 5
                        lon,   # param 6
                        alt )  # param 7
        cmds.add( cmd )                                     # コマンドの追加
        cmds.upload()
        
    elif command == 16:     # MAV_CMD_NAV_WAYPOINT
        cmd = Command(  0,                                           # system id
                        0,                                           # component id
                        1,                                           # seq
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   #frame
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,    # MAV_CMD
                        0,   # current
                        1,   # auto continue
                        0,   # param 1
                        0,   # param 2
                        0,   # param 3
                        0,   # param 4
                        lat,   # param 5
                        lon,   # param 6
                        alt )  # param 7
        cmds.add( cmd )                                     # コマンドの追加
        cmds.upload()

    elif command == 21:     # MAV_CMD_NAV_LAND
        cmd = Command(  0,                                           # system id
                        0,                                           # component id
                        1,                                           # seq
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   #frame
                        mavutil.mavlink.MAV_CMD_NAV_LAND,    # MAV_CMD
                        0,   # current
                        1,   # auto continue
                        0,   # param 1
                        0,   # param 2
                        0,   # param 3
                        0,   # param 4
                        lat,   # param 5
                        lon,   # param 6
                        alt )  # param 7
        cmds.add( cmd )                                     # コマンドの追加
        cmds.upload()

            

vRoberT.disarm()
vRoberT.mode = VehicleMode("MANUAL")
vRoberT.wait_for_mode("MANUAL")


vRoberT.armed = True
vRoberT.arm()
print("ARMED.")

vRoberT.mode = VehicleMode("AUTO")
vRoberT.wait_for_mode("AUTO")
print("mode = AUTO")

# print("land wait ...")

