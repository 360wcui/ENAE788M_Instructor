/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 *
 * from https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#define FREQ 20.0
#define EPS 0.01
#define DURATION 1.0
#define ENABLE_ARM_DURATION 1.0

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::PositionTarget pose_vel;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

bool closeEnough(geometry_msgs::Point current, geometry_msgs::Point target) {
    return abs(current.x - target.x) < EPS && abs(current.y - target.y) < EPS && abs(current.z - target.z) < EPS;
}

geometry_msgs::Point createTarget(int x, int y, int z) {
    geometry_msgs::Point target;
    target.x = x;
    target.y = y;
    target.z = z;
    return target;

}

geometry_msgs::Point foo(geometry_msgs::Point current_position, geometry_msgs::Point target, int* count, bool* reachedTarget, ros::Time* target_reached_time) {
    geometry_msgs::Point next_position;
    next_position = target;

    if (closeEnough(current_position, target)){
        if (!*reachedTarget) {
            *reachedTarget = true;
            *target_reached_time = ros::Time::now();
            ROS_INFO("step %d target reached.", *count);
            std::cout <<  *target_reached_time << std::endl;
        } else if (ros::Time::now() - *target_reached_time > ros::Duration(DURATION)) {
            std::cout << ros::Time::now() << " last time" << *target_reached_time << std::endl;
            ROS_INFO("step %d complete.", *count);
            std::cout << "current posiiton " << current_position << std::endl;
            *reachedTarget = false;
            *count = (*count + 1) % 4;
        }
     }
     return next_position;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

<<<<<<< HEAD
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber pose_sub = nh.subscribe("mavros/local_position/pose", 1000, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
=======
    //ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    //ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
>>>>>>> 0c23a39cd5b7a0c982b850b678f00d99ac630a6d
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(FREQ);

    // wait for FCU connection
    //while(ros::ok() && !current_state.connected){
    //    ros::spinOnce();
    //    rate.sleep();
    //}

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
<<<<<<< HEAD
    pose.pose.position.z = 10;

    geometry_msgs::Point target0 = createTarget(0, 0, 10);
    geometry_msgs::Point target1 = createTarget(0, 10, 10);
    geometry_msgs::Point target2 = createTarget(0, 10, 25);
    geometry_msgs::Point target3 = createTarget(-5, 10, 25);
=======
    pose.pose.position.z = 2;
>>>>>>> 0c23a39cd5b7a0c982b850b678f00d99ac630a6d

    //send a few setpoints before starting
    //for(int i = 200; ros::ok() && i > 0; --i){
    //    local_pos_pub.publish(pose);
    //    ros::spinOnce();
    //    rate.sleep();
    //}

<<<<<<< HEAD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
=======

   
    //pose_vel.coordinate_frame = pose_vel.FRAME_LOCAL_NED;
    //pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW | pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;


    //mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";
>>>>>>> 0c23a39cd5b7a0c982b850b678f00d99ac630a6d

    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time target_reached_time = ros::Time::now();
    bool reachedTarget = false;

    int count = 0;
    ROS_INFO("athena takes off");
    while(ros::ok()){
<<<<<<< HEAD
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
//                ROS_INFO("trying to arm call2");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (count == 0) {
            pose.pose.position = foo(current_pose.pose.position, target0, &count, &reachedTarget, &target_reached_time);
        }

        if (count == 1) {
            pose.pose.position = foo(current_pose.pose.position, target1, &count, &reachedTarget, &target_reached_time);
=======
        //if( current_state.mode != "OFFBOARD" &&
        //    (ros::Time::now() - last_request > ros::Duration(5.0))){
        //    if( set_mode_client.call(offb_set_mode) &&
        //        offb_set_mode.response.mode_sent){
        //        ROS_INFO("Offboard enabled");
        //    }
        //    last_request = ros::Time::now();
        //} else {
        //    if( !current_state.armed &&
        //        (ros::Time::now() - last_request > ros::Duration(5.0))){
        //        if( arming_client.call(arm_cmd) &&
        //            arm_cmd.response.success){
        //            ROS_INFO("Vehicle armed");
        //        }
        //       last_request = ros::Time::now();
        //    }
        //}
        

        if (count<100){
           pose.pose.position.x = -1.5;
           pose.pose.position.y = 0;
           pose.pose.position.z = 2;
        }
        else if (count<200){
           pose.pose.position.x = 1.5;
           pose.pose.position.y = 0;
           pose.pose.position.z = 2;
>>>>>>> 0c23a39cd5b7a0c982b850b678f00d99ac630a6d
        }

        if (count == 2) {
            pose.pose.position = foo(current_pose.pose.position, target2, &count, &reachedTarget, &target_reached_time);
        }

        if (count == 3) {
            pose.pose.position = foo(current_pose.pose.position, target3, &count, &reachedTarget, &target_reached_time);
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
<<<<<<< HEAD
=======
        count++;
	std::cout << count << std::endl;

>>>>>>> 0c23a39cd5b7a0c982b850b678f00d99ac630a6d
    }

    return 0;
}
