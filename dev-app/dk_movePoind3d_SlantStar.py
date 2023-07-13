#
#   filename    :   dk_movePoint3d_SlantStar.py
#   description :   ３次元のポイント移動
#                   傾けた☆
#
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

##### 接続
vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60 )
print("Connected.")

##### GUIDEDモード
vehicle.mode = VehicleMode("GUIDED")
vehicle.wait_for_mode("GUIDED")
print("mode = GUIDED")

##### ARM
vehicle.armed = True
vehicle.arm()
print("ARMED.")

### 離陸し、5mまで上昇
try:
    vehicle.wait_for_armable()
    vehicle.wait_for_mode("GUIDED")
    vehicle.arm()

    time.sleep(1)

    vehicle.wait_simple_takeoff( 10, timeout=30 )
    print("takeoff. alt=10m")

except TimeoutError as takeoffError:
    print("Takeoff is timeout.")

# 正五角形
#        x(E)     y(N)      z(D)
# p1     1.0      0.0       -1.0
# p2     0.31     0.95      -1.0
# p3    -0.81     0.59      -0.5
# p4    -0.81    -0.59      -1.5
# p5     0.31    -0.95      -0.5

posN = [  0.00,  0.59, -0.95,  0.95, -0.59,  0.00 ]   # y
posE = [  1.00, -0.81,  0.31,  0.31, -0.81,  1.00 ]   # x
posD = [ -1.00, -1.00, -0.50, -1.50, -0.50, -1.00 ]   # z

posMag = 10.0       # 倍率(m)

TargetMargin = 0.2      # 移動完了判断のマージン
MoveTimeout = 30        # 移動完了までのタイムアウト時間(s)

print("start.")
for N, E, D in zip( posN, posE, posD ):
    targetPosN = N * posMag
    targetPosE = E * posMag
    targetPosD = D * posMag

    msg = BuildSetPositionMessage( targetPosN, targetPosE, targetPosD )
    for x in range( 0, MoveTimeout ):
        currentPos  = vehicle.location.local_frame
        ArriveNorth = IsArrive( targetPosN, currentPos.north, TargetMargin )
        ArriveEast  = IsArrive( targetPosE, currentPos.east,  TargetMargin )
        ArriveDown  = IsArrive( targetPosD, currentPos.down,  TargetMargin )
        if ArriveNorth and ArriveEast and ArriveDown:
            break

        vehicle.send_mavlink( msg )
        time.sleep( 1 )

print("done.")
time.sleep( 5 )

# RTL
vehicle.mode = VehicleMode("RTL")
vehicle.wait_for_mode("RTL")
print("mode = RTL")


