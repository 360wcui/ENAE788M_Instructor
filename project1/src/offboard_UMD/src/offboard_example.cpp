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

int FREQ = 10;

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
    return abs(current.x - target.x) < 0.01 && abs(current.y - target.y) < 0.01 && abs(current.z - target.z) < 0.01;
}

geometry_msgs::Point createTarget(int x, int y, int z) {
    geometry_msgs::Point target;
    target.x = x;
    target.y = y;
    target.z = z;
    return target;

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber pose_sub = nh.subscribe("mavros/local_position/pose", 1000, pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 10;

    geometry_msgs::Point target0 = createTarget(0, 0, 10);
    geometry_msgs::Point target1 = createTarget(0, 10, 10);
    geometry_msgs::Point target2 = createTarget(0, 10, 25);
    geometry_msgs::Point target3 = createTarget(-5, 10, 25);

    //send a few setpoints before starting
    for(int i = 200; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }



    //pose_vel.coordinate_frame = pose_vel.FRAME_LOCAL_NED;
    //pose_vel.type_mask =  pose_vel.IGNORE_AFX | pose_vel.IGNORE_AFY | pose_vel.IGNORE_AFZ | pose_vel.FORCE | pose_vel.IGNORE_YAW | pose_vel.IGNORE_PX | pose_vel.IGNORE_PY | pose_vel.IGNORE_PZ;


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time target_reached_time = ros::Time::now();

    int count = 0;
    ROS_INFO("athena takes off");
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
//            ROS_INFO("trying to arm %d,  %.3f, %.3f, %.3f", current_state.armed, ros::Time::now() - last_request, last_request, ros::Time::now());
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

        if (current_state.armed) {
            bool reachedTarget = false;
             if (count == 0){
                if (!closeEnough(current_pose.pose.position, target0)){
                    pose.pose.position = target0;
                 } else{
                    ROS_INFO("step0 complete.");
                     std::cout << "current pose " << current_pose.pose.position << std::endl;
                    count = 1;
                 }
            }
//
//            if (count == 0) {
//                if (!closeEnough(current_pose.pose.position, target0)){
//                    pose.pose.position = target0;
//                 } else{
//                    if (!reachedTarget) {
//                        reachedTarget = true;
//                        target_reached_time = ros::Time::now();
//                        ROS_INFO("step0 target reached.");
//                        pose.pose.position = target0;
//                    } else if (ros::Time::now() - last_request > ros::Duration(1.0)) {
//                        ROS_INFO("step0 complete.");
//                        std::cout << "current pose " << current_pose.pose.position << std::endl;
//                        reachedTarget = false;
//                        count = 1;
//                    }
//                 }
//            }

            if (count == 1){
                if (!closeEnough(current_pose.pose.position, target1)){
                    pose.pose.position = target1;
                 } else{
                    ROS_INFO("step1 complete.");
                     std::cout << "current pose " << current_pose.pose.position << std::endl;
                    count = 2;
                 }
            }

            if (count == 2){
                if (!closeEnough(current_pose.pose.position, target2)){
                    pose.pose.position = target2;
                 } else{
                    ROS_INFO("step2 complete.");
                     std::cout << "current pose " << current_pose.pose.position << std::endl;
                    count = 3;
                 }
            }

            if (count == 3){
                if (!closeEnough(current_pose.pose.position, target3)){
                    pose.pose.position = target3;
                 } else{
                    ROS_INFO("step3 complete. repeat.");
                     std::cout << "current pose " << current_pose.pose.position << std::endl;
                    count = 0;
                 }
            }


            //float phase = ((float)count/40);
            //pose_vel.header.stamp = ros::Time::now();
            //pose_vel.yaw_rate = 0;
            //pose_vel.velocity.x = 1.5*sin(phase);
            //pose_vel.velocity.y = 0;
            //pose_vel.velocity.z = 0;

            //local_pos_pub_mavros.publish(pose_vel);

            local_pos_pub.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
//        count++;
    }

    return 0;
}
