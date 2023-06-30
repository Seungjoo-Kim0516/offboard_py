#! /usr/bin/env python3

import rospy
import os
import rosbag
import math
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL
from sensor_msgs.msg import Range

# bag 파일 저장하려면 ! /usr/bin/env python -> ! /usr/bin/env python3 로 바꿔줘야 됨
bag = None

current_state = State()
current_pose = PoseStamped()
current_vel = TwistStamped()
current_range = Range()

cnt=0.0
cnt1=0.0
new_cnt=0.0
new_cnt_x = 0.0
new_cnt_y = 0.0
a=0.0
b=0.0
r=3.0

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def vel_cb(msg):
    global current_vel
    current_vel = msg

def range_cb(msg):
    global current_range
    current_range = msg


if __name__ == "__main__":
    rospy.init_node("offb_node")

    # @@@@@ bag파일 관련 설정
    if not os.path.exists("BagFiles"):
        os.makedirs("BagFiles")
    home_dir = os.path.expanduser("~")
    bag_file = os.path.join(home_dir + "/BagFiles", "example.bag")
    bag = rosbag.Bag(bag_file, 'w')
    # 바탕화면의 BagFiles라는 폴더에 example이라는 bag파일을 저장하겠다는 의미

    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)
    local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb)
    local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, callback=vel_cb)
    local_range_sub = rospy.Subscriber("mavros/distance_sensor/lidarlite_pub", Range, callback=range_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/land")
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    # 1초에 20번 -> 0.05초에 1번

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    #비행 경로 initialize
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break

        # local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    manual_set_mode = SetModeRequest()
    manual_set_mode.custom_mode = 'MANUAL'

    last_req = rospy.Time.now()
    real_start_time=0.0


    # bag 파일 저장하려면 try except 구문 각주 해제 + tab
    try:
        while not rospy.is_shutdown():
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
                    real_start_time = rospy.get_time()
            else:
                if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                    if set_mode_client.call(offb_set_mode).mode_sent == True:
                        rospy.loginfo("OFFBOARD mode enabled")
                    last_req = rospy.Time.now()
                    real_start_time = rospy.get_time()

            # @@@@@@@@ 저장하고자 하는 bag 파일에서 record 하는 토픽 설정
            bag.write('mavros/local_position/pose', current_pose)
            bag.write('mavros/setpoint_position/local', pose)


            # #@@@@@@@@실린더 비행
            # pose.pose.position.x = r*math.cos(rospy.get_time()-real_start_time)
            # pose.pose.position.y = r*math.sin(rospy.get_time()-real_start_time)
            # pose.pose.position.z = rospy.get_time()-real_start_time

            # #@@@@@@@@8자 비행
            pose.pose.position.x = r*math.sin(rospy.get_time()-real_start_time)
            pose.pose.position.y = r*math.sin(rospy.get_time()-real_start_time)*math.cos(rospy.get_time()-real_start_time)
            pose.pose.position.z = 3

            local_pos_pub.publish(pose)

            rate.sleep()
            pass
    except rospy.ROSInterruptException as e:
       bag.close()

