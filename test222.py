#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    num_drones = 3

    state_subs = []
    local_pos_pubs = []
    arming_clients = []
    set_mode_clients = []
    drones_pose_subs = []

    for i in range(num_drones):
        state_subs.append(rospy.Subscriber("/uav{}/mavros/state".format(i), State, callback=state_cb))
        local_pos_pubs.append(rospy.Publisher("/uav{}/mavros/setpoint_position/local".format(i), PoseStamped, queue_size=10))
        arming_clients.append(rospy.ServiceProxy("/uav{}/mavros/cmd/arming".format(i), CommandBool))
        set_mode_clients.append(rospy.ServiceProxy("/uav{}/mavros/set_mode".format(i), SetMode))
        drones_pose_subs.append(rospy.Subscriber("/uav{}/mavros/local_position/pose".format(i), PoseStamped))

    rospy.init_node('formation_control', anonymous=True)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    # 목표 지점 설정
    target_x = 10.0
    target_y = 10.0
    target_z = 3.0

    # 팔로워 드론과의 간격 설정
    follower_spacing = 6.0

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            for i in range(num_drones):
                if set_mode_clients[i].call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD 모드 활성화")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                for i in range(num_drones):
                    if arming_clients[i].call(arm_cmd).success:
                        rospy.loginfo("uav {} 비행 준비 완료.".format(i))
                last_req = rospy.Time.now()

            # 리더 드론과 팔로워 드론들의 현재 위치 확인
            leader_pose = drones_pose_subs[0]
            follower1_pose = drones_pose_subs[1]
            follower2_pose = drones_pose_subs[2]

            # 리더 드론과 팔로워 드론들의 이동 명령 생성
            leader_cmd = PoseStamped()
            leader_cmd.pose.position.x = target_x
            leader_cmd.pose.position.y = target_y
            leader_cmd.pose.position.z = target_z

            follower1_cmd = PoseStamped()
            follower1_cmd.pose.position.x = target_x -5.12
            follower1_cmd.pose.position.y = target_y - 3
            follower1_cmd.pose.position.z = target_z

            follower2_cmd = PoseStamped()
            follower2_cmd.pose.position.x = target_x -5.12
            follower2_cmd.pose.position.y = target_y +3
            follower2_cmd.pose.position.z = target_z

            # 이동 명령을 발행
            local_pos_pubs[0].publish(leader_cmd)
            local_pos_pubs[1].publish(follower1_cmd)
            local_pos_pubs[2].publish(follower2_cmd)

        rate.sleep()
