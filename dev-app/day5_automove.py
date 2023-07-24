from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Vehicle
from pymavlink import mavutil
import time
import mpreader
import math

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def AutoMove( connection, fileName, initialMode, isBlock ):

    vheicle = connect( connection, wait_ready=True, timeout=60 )      # rover taigan
    print("Connected.", connection, fileName)

    vheicle.parameters[ 'WP_RADIUS' ] = 2   # wpマージン(m)

    # ##### ミッションの内容（一度DLして cmds を初期化しないとうまくいかない？）
    cmds = vheicle.commands
    cmds.download()
    cmds.wait_ready()

    ( commands, lats, lons, alts ) = mpreader.mpReader( fileName )

    for command, lat, lon, alt in zip( commands, lats, lons, alts ):
        print( command, lat, lon, alt )

    cmds = vheicle.commands
    cmds.clear()                                            # 現在のミッションをクリア
    cmds.upload()
    cmds.wait_ready()

    latestLat = -1
    latestLon = -1
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

            latestLat = lat
            latestLon = lon

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

   # 実行
    vheicle.disarm()
    vheicle.mode = VehicleMode( initialMode )
    vheicle.wait_for_mode( initialMode )

    vheicle.armed = True
    vheicle.arm()
    print("ARMED.")

    vheicle.mode = VehicleMode("AUTO")
    vheicle.wait_for_mode("AUTO")
    print("mode = AUTO")

    print( "latest point:", latestLat, latestLon )

# print("land wait ...")
    if isBlock == False:
        return

    target = LocationGlobalRelative( latestLat, latestLon, 0 )
    while True:
        currenntPos = vheicle.location.global_relative_frame
        lastMeters = get_distance_metres( currenntPos, target )

        if lastMeters < 5:
            break

        print("last :", lastMeters )
        time.sleep( 1 ) 


    # msg = vheicle.recv_match(type='MISSION_STATE',blocking=True)
    # if not msg:
    #     print("not support")
    #     return
    # if msg.get_type() == "BAD_DATA":
    #     if mavutil.all_printable(msg.data):
    #         print(msg.data)
    # else:
    #     #Message is valid
    #     # Use the attribute
    #     print('Mode: %s' % msg.mode)
