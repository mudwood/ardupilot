from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

# # ホームロケーション
# # vehicle.home_location に値がセットされるまで download を繰り返す
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    if not vehicle.home_location:
        print("Waiting for home location ...")

# ホームロケーションの取得完了
print("Home location: %s" % vehicle.home_location )

Location1 = LocationGlobalRelative( vehicle.home_location.lat + 1.0, vehicle.home_location.lon + 0.0, 20)
Location2 = LocationGlobalRelative( vehicle.home_location.lat + 1.0, vehicle.home_location.lon + 1.0, 20)
Location3 = LocationGlobalRelative( vehicle.home_location.lat + 0.0, vehicle.home_location.lon + 1.0, 20)

### 座標指定の移動
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
print("mode = GUIDED")

vehicle.armed = True
vehicle.arm()
print("ARMED.")

# 離陸
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    vehicle.arm()

    time.sleep(1)

    vehicle.wait_simple_takeoff( 20, timeout=20)
    print("takeoff. alt=10m")

except TimeoutError as takeoffError:
    print("Takeoff is timeout.")

# 目標の緯度経度、高度設定
HomeLocation = vehicle.home_location
print("Home : %s" % HomeLocation )

# aLocation = LocationGlobalRelative( -34.364114, 149.166022, 20)
# aLocation = LocationGlobalRelative( vehicle.home_location.lat + 1.0, vehicle.home_location.lon, 20)

# 移動
#vehicle.simple_goto( aLocation )
#time.sleep(5)

vehicle.simple_goto( Location1 )
time.sleep( 3 )
vehicle.mode = VehicleMode("LOITER")
vehicle.wait_for_mode("LOITER")
time.sleep( 3 )

vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
vehicle.simple_goto( Location2 )
time.sleep( 3 )
vehicle.mode = VehicleMode("LOITER")
vehicle.wait_for_mode("LOITER")
time.sleep( 3 )

vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
vehicle.simple_goto( Location3 )
time.sleep( 3 )
vehicle.mode = VehicleMode("LOITER")
vehicle.wait_for_mode("LOITER")
time.sleep( 3 )

# RTL
vehicle.mode = VehicleMode("RTL")
vehicle.wait_for_mode("RTL")
print("mode = RTL")


# # ホームロケーション
# # vehicle.home_location に値がセットされるまで download を繰り返す
# while not vehicle.home_location:
#     cmds = vehicle.commands
#     cmds.download()
#     cmds.wait_ready()

#     if not vehicle.home_location:
#         print("Waiting for home location ...")

# # ホームロケーションの取得完了
# print("Home location: %s" % vehicle.home_location )

# # RTL時の高さパラメータ
# print("Param: %s" % vehicle.parameters['RTL_ALT'])

# # RTLパラメータのセット
# vehicle.parameters['RTL_ALT'] = 1500

# # RTL時の高さパラメータ
# print("Param: %s" % vehicle.parameters['RTL_ALT'])

# # パラメータ全表示
# for key, value in vehicle.parameters.items():
#     print("Key:%s, Values:%s" % (key, value))

