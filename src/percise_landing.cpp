#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <ros/publisher.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>




class Lander
{
  public: 

    Lander(std::string AprilFrame, ros::NodeHandle nh_){
    
    ap_tag_frame = AprilFrame;
    target_reached = false;
    node_handle_ = nh_;
    tag_loc_pub = nh_.advertise<geometry_msgs::PoseStamped>("/aprilTag_landing_location",10);
    transform.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q(0,0,0,1);
    transform.setRotation(q);
    
    local_set_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    
    }
    
    std::string ap_tag_frame;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped landing_target;
    geometry_msgs::PoseStamped current_pose;
    tf::StampedTransform transform;
    
    ros::Publisher tag_loc_pub;
    ros::NodeHandle node_handle_;
    
    ros::Publisher local_set_pos_pub;
    
    
    bool target_reached;
    
    void SetLandingTarget(const tf2_msgs::TFMessageConstPtr& TFmsg);
    
    void EngageLanding(geometry_msgs::PoseStamped ApLTmsg);
    
    void SetCurrentPose(geometry_msgs::PoseStamped Pmsg);
   

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
    

    
    Lander AprilTagLander(apriltag_frame,nh);
    
    ros::Subscriber TFsub = nh.subscribe("/tf", 10, &Lander::SetLandingTarget, &AprilTagLander);
    ros::Subscriber AprilTag_location_sub = nh.subscribe("/aprilTag_landing_location", 10, &Lander::EngageLanding, &AprilTagLander);
    ros::Subscriber CurrentPose_sub = nh.subscribe("/mavros/local_position/pose", 10, &Lander::SetCurrentPose, &AprilTagLander);
    

    ros::Rate rate(20.0);
    while(ros::ok()){
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
    
    
    local_set_pos_pub.publish(Target_Pose);
    


}

void Lander::SetCurrentPose(geometry_msgs::PoseStamped Pmsg){

   current_pose = Pmsg;

}  
    
    
    
