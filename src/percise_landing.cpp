#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ParamPush.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <ros/publisher.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>




class Lander
{
  public: 

    Lander(std::string AprilFrame, ros::NodeHandle nh_, int channel){
    
    ap_tag_frame = AprilFrame;
    target_reached = false;
    offboard = false;
    target_found = false;
    radio = false;
    node_handle_ = nh_;
    tag_loc_pub = nh_.advertise<geometry_msgs::PoseStamped>("/aprilTag_landing_location",10);
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    transform.setRotation(q);
    
    local_set_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_set_pose_raw_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    landing_client = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    
    parameter_set_client = nh_.serviceClient<mavros_msgs::ParamPush>("mavros/param/push");
    
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;
    
    vel.x = 0.001;
    vel.y = 0.001;
    vel.z = 0.001;
    
    radio_channel = channel;


    
    }
    
    std::string ap_tag_frame;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped landing_target;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::Vector3 vel;
    tf::StampedTransform transform;
    mavros_msgs::State current_state;
    
    ros::Publisher tag_loc_pub;
    ros::NodeHandle node_handle_;
    
    ros::Publisher local_set_pos_pub;
    ros::Publisher local_set_pose_raw_pub;
    
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient landing_client;
    ros::ServiceClient parameter_set_client;
    
    mavros_msgs::CommandTOL land_cmd;
    mavros_msgs::PositionTarget pose_raw;
    mavros_msgs::ParamPush param_push_msg;
    
    int radio_channel;
    
    bool target_reached;
    
    bool offboard;
    
    bool target_found;

    bool radio;
    
    void SetLandingTarget(const tf2_msgs::TFMessageConstPtr& TFmsg);
    
    void EngageLanding(geometry_msgs::PoseStamped ApLTmsg);
    
    void SetCurrentPose(geometry_msgs::PoseStamped Pmsg);
    
    void FcuState(mavros_msgs::State Statemsg);

    void SetRadio(mavros_msgs::RCIn RCmsg);   
   

};


int main(int argc, char **argv)
{

    ros::init(argc, argv,"percise_landing_apriltag");
    ros::NodeHandle nh;
    
    std::string apriltag_frame;   
    nh.param<std::string>("apriltag_frame",apriltag_frame,"tagTF");
    nh.getParam("apriltag_frame",apriltag_frame);
 
    std::string detection_topic;   
    nh.param<std::string>("detection_topic",detection_topic,"tag_detections");
    nh.getParam("detection_topic",detection_topic);
    
    int channel_num;
    nh.param<int>("channel_num",channel_num,9);
    nh.getParam("channel_num",channel_num);
    
    float mavros_param_MPC_XY_VEL_MAX;
    nh.param<float>("/mavros/param/MPC_XY_VEL_MAX",mavros_param_MPC_XY_VEL_MAX,0.1);
    nh.getParam("/mavros/param/MPC_XY_VEL_MAX",mavros_param_MPC_XY_VEL_MAX);

    
    Lander AprilTagLander(apriltag_frame,nh,channel_num);
    
    ros::Subscriber TFsub = nh.subscribe("/tf", 10, &Lander::SetLandingTarget, &AprilTagLander);
    ros::Subscriber AprilTag_location_sub = nh.subscribe("/aprilTag_landing_location", 10, &Lander::EngageLanding, &AprilTagLander);
    ros::Subscriber CurrentPose_sub = nh.subscribe("/mavros/local_position/pose", 10, &Lander::SetCurrentPose, &AprilTagLander);
    ros::Subscriber radio_sub = nh.subscribe("/mavros/rc/in", 10, &Lander::SetRadio, &AprilTagLander);
    

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 100, &Lander::FcuState, &AprilTagLander);

   
    

    ros::Rate rate(20.0);
    
    while(ros::ok() && !AprilTagLander.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    /*for(int i = 100; ros::ok() && i > 0; --i){
        AprilTagLander.local_set_pos_pub.publish(AprilTagLander.landing_target);
        ros::spinOnce();
        rate.sleep();
    }*/

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    

    ros::Time last_request = ros::Time::now();
    
    
    
    while(ros::ok()){
    
    
         if (AprilTagLander.radio && !AprilTagLander.offboard && AprilTagLander.current_state.mode != "OFFBOARD" && !AprilTagLander.target_reached && (ros::Time::now() - last_request > ros::Duration(5.0))){
            
            ROS_INFO("Enabling Offboard");
            
            if(AprilTagLander.set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            
            last_request = ros::Time::now();
         
         } 
         

        if (AprilTagLander.current_state.mode == "OFFBOARD" && AprilTagLander.target_reached && (ros::Time::now() - last_request > ros::Duration(5.0))){

            if (AprilTagLander.landing_client.call(AprilTagLander.land_cmd) && AprilTagLander.land_cmd.response.success){
                ROS_INFO("Goal Reached Landing");
            }
            
            last_request = ros::Time::now();
        }
        
        if (AprilTagLander.current_state.mode == "OFFBOARD"){
        		
        		if (!AprilTagLander.offboard){
        			ROS_INFO("setting offboard to true");
        			AprilTagLander.offboard = true;
        		}
        
        }
        
        
    
    
    
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;

    
}



void Lander::SetLandingTarget(const tf2_msgs::TFMessageConstPtr& tfmsg){


    try{
      listener.lookupTransform("/local_origin", ap_tag_frame, ros::Time(0), transform);
    }
    
    catch (tf::TransformException ex){
      return;
      //ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
    
    target_found = true;
    
    landing_target.header.stamp = ros::Time::now();
    landing_target.header.frame_id = "/local_origin";
    
    landing_target.pose.position.x = transform.getOrigin().getX();
    landing_target.pose.position.y = transform.getOrigin().getY();
    landing_target.pose.position.z = transform.getOrigin().getZ();
    
    tf::Quaternion quat_tf= transform.getRotation();
    geometry_msgs::Quaternion quat_msg;
    
    tf::quaternionTFToMsg(quat_tf,quat_msg);
    
    

    
    landing_target.pose.orientation = quat_msg; 
    
    tag_loc_pub.publish(landing_target);

}  
    
void Lander::EngageLanding(geometry_msgs::PoseStamped uavPosemsg){

    geometry_msgs::PoseStamped Target_Pose;
    
    Target_Pose.header.stamp = ros::Time::now();
    Target_Pose.header.frame_id = "/local_origin";
    
    Target_Pose.pose = current_pose.pose; 
    Target_Pose.pose.position.x = landing_target.pose.position.x;
    Target_Pose.pose.position.y = landing_target.pose.position.y;
    
    pose_raw.position = Target_Pose.pose.position;
    pose_raw.velocity = vel;
    pose_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    
    //local_set_pose_raw_pub.publish(pose_raw);
    if (radio){
      local_set_pos_pub.publish(Target_Pose);
    }


}

void Lander::SetCurrentPose(geometry_msgs::PoseStamped Pmsg){

   current_pose = Pmsg;
   
   if (target_found && (abs(current_pose.pose.position.x - landing_target.pose.position.x) < 0.01) && (abs(current_pose.pose.position.y - landing_target.pose.position.y) < 0.01) ){
   		ROS_INFO("Setting target_reached to true");
   		target_reached = true;
   }


}  


void Lander::FcuState(mavros_msgs::State Statemsg){

   current_state = Statemsg;

}

void Lander::SetRadio(mavros_msgs::RCIn RCmsg){

   if (RCmsg.channels[radio_channel] == 2006 && !radio){
      radio = true;
      std::cout<<"Radio activated"<<std::endl;
      node_handle_.setParam("/mavros/param/MPC_XY_VEL_MAX",0.1);
      parameter_set_client.call(param_push_msg);
      
   }
   else if (radio && RCmsg.channels[radio_channel] != 2006){
      radio = false;
      std::cout<<"Radio deactivated"<<std::endl;
      node_handle_.setParam("/mavros/param/MPC_XY_VEL_MAX",12.0);
      parameter_set_client.call(param_push_msg);
   }
   
}     
    
    
