#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import math
import tf



def Set_Odom(msg):
    global pos,front
    pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # quaternion to euler
    (Alpha, Beta, Gamma) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                        msg.pose.pose.orientation.y,
                                                                        msg.pose.pose.orientation.z,
                                                                        msg.pose.pose.orientation.w])
    front = Rad2Deg(Gamma)

def Vel_pub(vel):
    twist = Twist()
    twist.linear.x = vel[0]
    twist.angular.z = vel[1]
    pub_vel.publish(twist)


pub_vel = rospy.Publisher('cmd_vel/man_1', Twist, queue_size=1)
sub_odom = rospy.Subscriber("odom/man_1", Odometry, Set_Odom)
run_rate = 5 # hz
vel_ahead = 1.5
vel_turn = 1.0
correction_para = 0.1
pos = [0,0]
front = 0
points = [[6,2.2], [1.8,2.2],[1.8,9.2],[1.8,2.2]]
points_counter = 0
run_state = 0
pre_points_counter = 1
pre_run_state = 1
error_angle = 5 # degrees
error_distance = 0.5 # meters


def Rad2Deg(angle):  # radius to degree
    return angle * 180 / math.pi

def calcu_dis(target_pos):
    global pos
    return math.sqrt(math.pow((target_pos[0]-pos[0]),2)+math.pow((target_pos[1]-pos[1]),2))

def calcu_ang(target_pos):
    global pos,front
    ang = Rad2Deg(math.atan2((target_pos[1] - pos[1]), (target_pos[0] - pos[0])))-front
    if ang>=180:
        return ang-360
    elif ang <= -180:
        return ang+360
    else:
        return ang

def X_speed(dis):
    return vel_ahead

def Z_speed(ang):
    if ang > 0:
        return vel_turn*correction_para
    elif ang < 0:
        return vel_turn*(-correction_para)
    else:
        return vel_turn*0

def run_points(data):
    global run_state, pre_run_state, points_counter, pre_points_counter

    # print status
    if points_counter != pre_points_counter or run_state != pre_run_state:
        print ("man 111 for {} state : {}".format(data[points_counter], run_state))
        pre_points_counter = points_counter
        pre_run_state = run_state

    # turn
    if run_state == 0:
        if error_angle <= abs(calcu_ang(data[points_counter])):
            # print ("ang : {}".format(calcu_ang(data[points_counter])))
            if calcu_ang(data[points_counter]) >= 0:
                Vel_pub([0,vel_turn])
            else:
                Vel_pub([0,-vel_turn])
        else:
            run_state = 1

    # go ahead
    if run_state == 1:
        if error_distance <= calcu_dis(data[points_counter]):
            # print ("dis : {}".format(abs(calcu_dis(data[points_counter]))))
            Vel_pub([X_speed(calcu_dis(data[points_counter])),Z_speed(calcu_ang(data[points_counter]))])
        else:
            if points_counter < len(data)-1:
                points_counter += 1
            else:
                points_counter = 0
            run_state = 0

def main():
    rospy.init_node('walking_man_1', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # rospy.loginfo("moving")
        global points
        run_points(points)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
