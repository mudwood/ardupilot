from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

##### functions
# description : mavlinkへ位置設定メッセージを投げる
def BuildSetPositionMessage( North, East, Down ):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,                                          # ブートからの時間（未使用
    0, 0,                                       # ターゲットシステム、コンポーネント
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,        # フレーム
    0b110111111000,                             # タイプマスク(use position)
    North, East, Down,                                  # X, Y, Z 位置（North, East, down
    0, 0, 0,                                    # X, Y, Z 速度 (m/s), 未使用
    0, 0, 0,                                    # X, Y, Z 加速度（未サポート
    0, 0)                                       # ヨー、ヨーレート
    return msg

# description : 到達チェック
def IsArrive( Target, Now, Margin ):
    if ( Target - Margin ) <= Now and Now <= ( Target + Margin ):
        Arrive = True
    else:
        Arrive = False
    return Arrive


##### body

#vehicle = connect('tcp:127.0.0.1:5780', wait_ready=True, timeout=60 )
vehicle = connect('udp:127.0.0.1:14571', wait_ready=True, timeout=60 )
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


##### GUIDEDモードでARM
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
print("mode = GUIDED")

vehicle.armed = True
vehicle.arm()
print("ARMED.")

### 離陸し、20mまで上昇
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    vehicle.arm()

    time.sleep(1)

    vehicle.wait_simple_takeoff( 20, timeout=20)
    print("takeoff. alt=20m")

except TimeoutError as takeoffError:
    print("Takeoff is timeout.")

# 正五角形
#        x(E)     y(N)
# p1     1.0      0.0
# p2     0.31     0.95
# p3    -0.81     0.59
# p4    -0.81    -0.59
# p5     0.31    -0.95

posN = [ 0.0, 0.59, -0.95, 0.95, -0.59, 0.0 ]
posE = [ 1.0, -0.81, 0.31, 0.31, -0.81, 1.0 ]
posD = -10.0
posMag = 10.0

TargetMargin = 0.2      # 移動完了判断のマージン
MoveTimeout = 30        # 移動完了までのタイムアウト時間(s)

print("start.")
for posN, posE in zip( posN, posE ):
    targetPosN = posN * posMag
    targetPosE = posE * posMag

    msg = BuildSetPositionMessage( targetPosN, targetPosE, posD )
    for x in range( 0, MoveTimeout ):
        currentPos  = vehicle.location.local_frame
        ArriveNorth = IsArrive( targetPosN, currentPos.north, TargetMargin )
        ArriveEast  = IsArrive( targetPosE, currentPos.east,  TargetMargin )
        if ArriveNorth and ArriveEast:
            break

        vehicle.send_mavlink( msg )
        time.sleep( 1 )

print("done.")
time.sleep( 5 )

# RTL
vehicle.mode = VehicleMode("RTL")
vehicle.wait_for_mode("RTL")
print("mode = RTL")


