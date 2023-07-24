from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Vehicle
from pymavlink import mavutil
import time

##### func

##### body

vQplane = connect('udp:127.0.0.1:14561', wait_ready=True, timeout=60 )      # quadplane
# vCopter = connect('udp:127.0.0.1:14571', wait_ready=True, timeout=60 )      # copter
# vBoat   = connect('udp:127.0.0.1:14581', wait_ready=True, timeout=60 )      # boat
# vRoverR = connect('udp:127.0.0.1:14591', wait_ready=True, timeout=60 )      # rover rinsetsu
# vRoberT = connect('udp:127.0.0.1:14601', wait_ready=True, timeout=60 )      # rover taigan

print("Connected.")

vQplane.parameters[ 'WP_RADIUS' ] = 5   # wpマージン(m)


# ##### ミッションの内容（一度DLして cmds を初期化しないとうまくいかない？）
cmds = vQplane.commands
cmds.download()
cmds.wait_ready()

# mission: MISSION_ITEM {target_system : 255, target_component : 0, seq : 1, frame : 3, command : 22, current : 0, autocontinue : 1, param1 : 0.0, param2 : 0.0, param3 : 0.0, param4 : 0.0, x : 0.0, y : 0.0, z : 50.0, mission_type : 0}
# mission: MISSION_ITEM {target_system : 255, target_component : 0, seq : 2, frame : 3, command : 16, current : 0, autocontinue : 1, param1 : 0.0, param2 : 0.0, param3 : 0.0, param4 : 0.0, x : 35.87850570678711, y : 140.3389129638672, z : 50.0, mission_type : 0}
# mission: MISSION_ITEM {target_system : 255, target_component : 0, seq : 3, frame : 3, command : 21, current : 0, autocontinue : 1, param1 : 0.0, param2 : 0.0, param3 : 0.0, param4 : 1.0, x : 0.0, y : 0.0, z : 0.0, mission_type : 0}
# cmd22 = MAV_CMD_NAV_TAKEOFF
# cmd16 = MAV_CMD_NAV_WAYPOINT
# cmd21 = MAV_CMD_NAV_LAND

# NRT to main
doCommand = [         22,         16,         21 ]
doLat     = [  35.760215,  35.878275,  35.878275 ]
doLon     = [ 140.379330, 140.338069, 140.338069 ]
doAlt     = [  50.0,       50.0,       50.0      ]

# main to NRT
# doCommand = [         22,         16,         21 ]
# doLat     = [  35.878275,  35.760215,  35.760215 ]
# doLon     = [ 140.338069, 140.379330, 140.379330 ]
# doAlt     = [ 100.0,      100.0,      100.0      ]

cmds = vQplane.commands
cmds.clear()                                            # 現在のミッションをクリア
cmds.upload()
cmds.wait_ready()

for command, lat, lon, alt in zip( doCommand, doLat, doLon, doAlt ):
    if command == 22:
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
    elif command == 16:
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
    elif command == 21:
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

            



################### ループ構造

# cmd = Command( 0,                                           # system id
#                0,                                           # component id
#                1,                                           # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   #frame
#                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    # MAV_CMD
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.760215,   # param 5
#                140.379330,   # param 6
#                50)  # param 7
# cmds.add( cmd )                                     # コマンドの追加
# cmds.upload()

# cmd = Command( 0,   # sys
#                0,   # component
#                2,   # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.878275,   # param 5
#                140.338069,  # param 6
#                50 ) # param 7

# cmds.add( cmd )
# cmds.upload()

# cmd = Command( 0,   # sys
#                0,   # component
#                3,   # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                mavutil.mavlink.MAV_CMD_NAV_LAND, 
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.878275,   # param 5
#                140.338069,  # param 6
#                50 ) # param 7

# cmds.add( cmd )
# cmds.upload()


vQplane.armed = True
vQplane.arm()
print("ARMED.")

vQplane.mode = VehicleMode("AUTO")
vQplane.wait_for_mode("AUTO")
print("mode = AUTO")

print("land wait ...")


# time.sleep( 30 )

# cmds.clear()                                            # 現在のミッションをクリア
# cmds.upload()
# cmds.wait_ready()

# cmd = Command( 0,                                           # system id
#                0,                                           # component id
#                1,                                           # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   #frame
#                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,    # MAV_CMD
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.878275,   # param 5
#                140.338069,  # param 6
#                50)  # param 7
# cmds.add( cmd )                                     # コマンドの追加
# cmds.upload()

# cmd = Command( 0,   # sys
#                0,   # component
#                2,   # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.878275,   # param 5
#                140.338069,   # param 6
#                50 ) # param 7

# cmds.add( cmd )
# cmds.upload()

# cmd = Command( 0,   # sys
#                0,   # component
#                3,   # seq
#                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#                mavutil.mavlink.MAV_CMD_NAV_LAND, 
#                0,   # current
#                1,   # auto continue
#                0,   # param 1
#                0,   # param 2
#                0,   # param 3
#                0,   # param 4
#                35.878275,   # param 5
#                140.338069,   # param 6
#                50 ) # param 7

# cmds.add( cmd )
# cmds.upload()


# vQplane.armed = True
# vQplane.arm()
# print("ARMED.")

# vQplane.mode = VehicleMode("AUTO")
# vQplane.wait_for_mode("AUTO")
# print("mode = AUTO")


