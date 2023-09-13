/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"
#include <geometry_msgs/Twist.h>

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

unitree_legged_msgs::HighCmd rosMsg2Cmd(const geometry_msgs::Twist::ConstPtr &msg)
{
    unitree_legged_msgs::HighCmd cmd;

    cmd.forwardSpeed = 0.0f;
    cmd.sideSpeed = 0.0f;
    cmd.rotateSpeed = 0.0f;
    cmd.bodyHeight = 0.0f;

    cmd.mode = 0;
    cmd.roll  = 0;
    cmd.pitch = 0;
    cmd.yaw = 0;

    cmd.forwardSpeed = msg->linear.x;
    cmd.sideSpeed = msg->linear.y;
    cmd.rotateSpeed = msg->angular.z;

    cmd.mode = 2;

    return cmd;
}

unitree_legged_msgs::HighCmd SendHighROS;

// timeout on last cmd_vel
ros::Time last_received_time;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    SendHighROS = rosMsg2Cmd(msg);
    last_received_time = ros::Time::now();
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm) {

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    TCmd SendHighLCM = {0};
    TState RecvHighLCM = {0};
    unitree_legged_msgs::HighState RecvHighROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    ros::Subscriber sub = n.subscribe("codog0/cmd_vel/smooth", 1, cmdVelCallback);

    SendHighROS.forwardSpeed = 0.0f;
    SendHighROS.sideSpeed = 0.0f;
    SendHighROS.rotateSpeed = 0.0f;
    SendHighROS.bodyHeight = 0.0f;

    SendHighROS.mode = 0;
    SendHighROS.roll  = 0;
    SendHighROS.pitch = 0;
    SendHighROS.yaw = 0;

    last_received_time = ros::Time::now();
    double time_elapsed = 0;

    while (ros::ok()){

        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("{%f, %f, %f, %d} \n",  RecvHighROS.forwardSpeed, RecvHighROS.sideSpeed, RecvHighROS.rotateSpeed, RecvHighROS.mode);
        
        time_elapsed = (ros::Time::now() - last_received_time).toSec();
        if (time_elapsed > 1.0) {
            SendHighROS.forwardSpeed = 0.0f;
            SendHighROS.sideSpeed = 0.0f;
            SendHighROS.rotateSpeed = 0.0f;
            SendHighROS.bodyHeight = 0.0f;

            SendHighROS.mode = 0;
            SendHighROS.roll  = 0;
            SendHighROS.pitch = 0;
            SendHighROS.yaw = 0;

            roslcm.Get(RecvHighLCM);
            RecvHighROS = ToRos(RecvHighLCM);
            // printf("{%f, %f, %f, %d} \n",  RecvHighROS.forwardSpeed, RecvHighROS.sideSpeed, RecvHighROS.rotateSpeed, RecvHighROS.mode);

            SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
            roslcm.Send(SendHighLCM);
            printf("timeout reached %f \n", time_elapsed);
        } else {
            SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
            roslcm.Send(SendHighLCM);
        }

        if (time_elapsed > 5.0) {
            ros::shutdown();
        }
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cmd_vel_sub");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    #ifdef SDK3_1
        aliengo::Control control(aliengo::HIGHLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::HighCmd, aliengo::HighState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;

        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
    
}