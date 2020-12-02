#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

float required_height = 2.0;
float required_x = 0.0;
float required_y = 0.0;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose, goal_pose;
bool goal=false;
bool goal_set=false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose= *msg;
    ROS_INFO("Current height = ");
    std::cout<<current_pose.pose.position.z<<std::endl;
    ROS_INFO("Goal Height");
    std::cout<<goal_pose.pose.position.z<<std::endl;
    if (current_pose.pose.position.z>=goal_pose.pose.position.z)
    {
        goal=true;
    }
    else
    {
        goal=false;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv,"offb_node");
    ros::NodeHandle nh;

    mavros_msgs::CommandTOL land_cmd;
    
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if(!goal_set){
            std::cout<<std::to_string(current_pose.pose.position.x)+" "+std::to_string(current_pose.pose.position.y)+" "+std::to_string(current_pose.pose.position.z)<<std::endl;
            goal_pose.pose.position.x = current_pose.pose.position.x + required_x;
            goal_pose.pose.position.y = current_pose.pose.position.y + required_y;
            goal_pose.pose.position.z = current_pose.pose.position.z + required_height;
            goal_pose.pose.orientation = current_pose.pose.orientation;
            ROS_INFO("Goal is set");
            goal_set=true;
        }
        std::cout<<"goal reached"<<std::endl;
        std::cout<<goal<<std::endl;
        if (current_state.mode != "OFFBOARD" && !goal &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO("Enabling Offboard");
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && !goal &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (current_state.mode == "OFFBOARD" && goal && (ros::Time::now() - last_request > ros::Duration(5.0))){

            if (landing_client.call(land_cmd) && land_cmd.response.success){
                ROS_INFO("Goal Reached Landing");
            } 

        } 



        local_pos_pub.publish(goal_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
