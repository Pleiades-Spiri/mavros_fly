#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ParamPush.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf2/buffer_core.h>
#include <tf/transform_broadcaster.h>
#include <ros/publisher.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>




class Lander
{
  public: 

    Lander(std::string AprilFrame, ros::NodeHandle nh_, int channel, double aVel, double TargTol, double PadTagD){
    
      ap_tag_frame = AprilFrame;
      target_reached = false;
      offboard = false;
      target_found = false;
      landing_target_set = false;
      Target_Pose_set = false;
      Pid_Set = false;
      radio = false;
      node_handle_ = nh_;
      tag_loc_pub = nh_.advertise<geometry_msgs::PoseStamped>("/aprilTag_landing_location",10);
      tag_loc_point_pub = nh_.advertise<geometry_msgs::Point>("/aprilTag_landing_point",10);
      transform.setOrigin(tf::Vector3(0,0,0));
      tf::Quaternion q(0,0,0,1);
      transform.setRotation(q);
      
      local_set_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
      local_set_pose_raw_pub = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
      setpoint_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
      
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


      approuch_vel = aVel;
      
      target_tol = TargTol;
      
      pad_target_x_del = PadTagD;

    
    }
    
    std::string ap_tag_frame;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped landing_target;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped Target_Pose;
    geometry_msgs::Vector3 vel;
    geometry_msgs::TwistStamped Target_Yaw;
    tf::StampedTransform transform;
    tf2_ros::Buffer buffer_;
    tf::TransformBroadcaster br;
    tf::Transform landing_offset_frame;


    mavros_msgs::State current_state;
    
    ros::Publisher tag_loc_pub;
    ros::Publisher tag_loc_point_pub;
    ros::NodeHandle node_handle_;
    
    ros::Publisher local_set_pos_pub;
    ros::Publisher local_set_pose_raw_pub;
    ros::Publisher setpoint_vel_pub;
    
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient landing_client;
    ros::ServiceClient parameter_set_client;
    
    mavros_msgs::CommandTOL land_cmd;
    mavros_msgs::PositionTarget pose_raw;
    mavros_msgs::ParamPush param_push_msg;
    mavros_msgs::PositionTarget pid_vel_target;
    
    float target_yaw;
    
    
    
    
    
    int radio_channel;

    double approuch_vel;
    
    double target_tol;
    
    double pad_target_x_del;
    
    bool target_reached;
    
    bool offboard;
    
    bool target_found;
    
    bool tag_visible;

    bool landing_target_set;

    bool Target_Pose_set;
    
    bool Pid_Set;

    bool radio;
    
    void SetLandingTarget(const tf2_msgs::TFMessageConstPtr& TFmsg);
    
    void SetTagVisible(apriltag_ros::AprilTagDetectionArray);
   
    void SetRawVel(mavros_msgs::PositionTarget PIDmsg);
    
    void EngageLanding(geometry_msgs::PoseStamped ApLTmsg);
    
    void SetCurrentPose(geometry_msgs::PoseStamped Pmsg);
    
    void FcuState(mavros_msgs::State Statemsg);

    void SetRadio(mavros_msgs::RCIn RCmsg);
    
    void Update();

   
   

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
    nh.param<float>("/mavros/param/MPC_XY_VEL_MAX",mavros_param_MPC_XY_VEL_MAX,0.5);
    nh.getParam("/mavros/param/MPC_XY_VEL_MAX",mavros_param_MPC_XY_VEL_MAX);
    
    float target_tolerance;
    nh.param<float>("target_tolerance",target_tolerance,0.1);
    nh.getParam("target_tolerance",target_tolerance);
    
    float pad_to_target_xdelta;
    nh.param<float>("pad_to_target_xdelta",pad_to_target_xdelta,0.0);
    nh.getParam("pad_to_target_xdelta",pad_to_target_xdelta);

    
    Lander AprilTagLander(apriltag_frame,nh,channel_num,mavros_param_MPC_XY_VEL_MAX,target_tolerance,pad_to_target_xdelta);
    
    ros::Subscriber TFsub = nh.subscribe("/tf", 10, &Lander::SetLandingTarget, &AprilTagLander);
    ros::Subscriber AprilTag_Detection = nh.subscribe("/tag_detections", 1, &Lander::SetTagVisible, &AprilTagLander);
    ros::Subscriber AprilTag_location_sub = nh.subscribe("/aprilTag_landing_location", 10, &Lander::EngageLanding, &AprilTagLander);
    ros::Subscriber CurrentPose_sub = nh.subscribe("/mavros/local_position/pose", 10, &Lander::SetCurrentPose, &AprilTagLander);
    ros::Subscriber radio_sub = nh.subscribe("/mavros/rc/in", 10, &Lander::SetRadio, &AprilTagLander);
    ros::Subscriber pid_sub = nh.subscribe("/pid/setpoint_raw/local", 10, &Lander::SetRawVel, &AprilTagLander);
    

    ros::Subscriber state_sub = nh.subscribe("mavros/state", 100, &Lander::FcuState, &AprilTagLander);

   
    

    ros::Rate rate(20.0);
    
    while(ros::ok() && !AprilTagLander.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    
    

    ros::Time last_request = ros::Time::now();
    
    
    
    while(ros::ok()){
    

         if (AprilTagLander.radio && !AprilTagLander.offboard && AprilTagLander.current_state.armed && AprilTagLander.current_state.mode != "OFFBOARD" && !AprilTagLander.target_reached && (ros::Time::now() - last_request > ros::Duration(5.0))){
            
            ROS_INFO("Enabling Offboard");
            
            if(AprilTagLander.set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            
            last_request = ros::Time::now();
         
         }
         else{
            
            if (!AprilTagLander.offboard){
              ROS_INFO("Vehicle not ready");
            } 
         
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
        
        AprilTagLander.Update();
        
        
    
    
    
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;

    
}



void Lander::SetLandingTarget(const tf2_msgs::TFMessageConstPtr& tfmsg){


    
    
    /*if (!tag_visible){
          
      return;
    }

    try{
      listener.lookupTransform("/fcu", ap_tag_frame, ros::Time(0), transform);
      target_found = true;
    
      landing_target.header.stamp = ros::Time::now();
      landing_target.header.frame_id = "fcu";
      
      landing_target.pose.position.x = transform.getOrigin().getX()-pad_target_x_del;
      landing_target.pose.position.y = transform.getOrigin().getY();
      landing_target.pose.position.z = transform.getOrigin().getZ();
      
      tf::Quaternion quat_tf= transform.getRotation();
      geometry_msgs::Quaternion quat_msg;
      
      tf::quaternionTFToMsg(quat_tf,quat_msg);
      
      

      
      landing_target.pose.orientation = quat_msg;

      landing_target_set = true;
      
      tag_loc_pub.publish(landing_target);
    }
    
    catch (tf::TransformException ex){
      return;
      //ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
    */
    
     
}  
    
void Lander::EngageLanding(geometry_msgs::PoseStamped uavPosemsg){

    if(Target_Pose_set & landing_target_set){
        Target_Pose.header.stamp = ros::Time::now();
        return;
    }
    
    if (landing_target_set){

      tf2_ros::TransformListener tfListener(buffer_);
      

      
      geometry_msgs::PoseStamped Target_Pose_fcu;
      //Target_Pose_fcu.header = landing_target.header;
      Target_Pose_fcu.header.stamp = ros::Time::now();
      Target_Pose_fcu.header.frame_id = "local_origin";
      
      //Target_Pose.pose = current_pose.pose; 
      Target_Pose_fcu.pose.position.x = landing_target.pose.position.x;
      Target_Pose_fcu.pose.position.y = landing_target.pose.position.y;
      try
      {
        buffer_.lookupTransform("fcu","local_origin", ros::Time::now(), ros::Duration(3.0));
        Target_Pose = buffer_.transform(landing_target, "local_origin");
        Target_Pose.pose.orientation = current_pose.pose.orientation;
        Target_Pose.pose.position.z = current_pose.pose.position.z;
        Target_Pose_set = true;
      }
      catch(tf::TransformException ex)
      {
        //ROS_ERROR("%s",ex.what());
      }

    }

    


}

void Lander::SetCurrentPose(geometry_msgs::PoseStamped Pmsg){

   current_pose = Pmsg;
   
   if (target_found && (abs(current_pose.pose.position.x - Target_Pose.pose.position.x) < target_tol) && (abs(current_pose.pose.position.y - Target_Pose.pose.position.y) < target_tol) ){
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
      //node_handle_.setParam("/mavros/param/MPC_XY_VEL_MAX",approuch_vel);
      //parameter_set_client.call(param_push_msg);
      
   }
   else if (radio && RCmsg.channels[radio_channel] != 2006){
      radio = false;
      offboard = false;
      target_reached = false;
      target_found = false;
      Pid_Set = false;
      std::cout<<"Radio deactivated"<<std::endl;
      //node_handle_.setParam("/mavros/param/MPC_XY_VEL_MAX",12.0);
      //parameter_set_client.call(param_push_msg);
      
      
   }
   
}

void Lander::SetTagVisible(apriltag_ros::AprilTagDetectionArray DectArr){

  if(DectArr.detections.size()>0)
  {
    tag_visible = true;
    landing_offset_frame.setOrigin( tf::Vector3(0.0, 0.0, pad_target_x_del) );
    landing_offset_frame.setRotation( tf::Quaternion(-0.5, 0.5, 0.5, 0.5) );
    br.sendTransform(tf::StampedTransform(landing_offset_frame, ros::Time::now(), ap_tag_frame ,"landingTF"));
    tf::StampedTransform transformTag;
    
    
    try{
      listener.lookupTransform("/fcu", "landingTF", ros::Time(0), transform);
      target_found = true;
    
      landing_target.header.stamp = ros::Time::now();
      landing_target.header.frame_id = "fcu";
      
      landing_target.pose.position.x = transform.getOrigin().getX();
      landing_target.pose.position.y = transform.getOrigin().getY();
      landing_target.pose.position.z = transform.getOrigin().getZ();
      
      tf::Quaternion quat_tf= transform.getRotation();

      target_yaw = tf::getYaw(quat_tf);

      
      geometry_msgs::Quaternion quat_msg;
      
      tf::quaternionTFToMsg(quat_tf,quat_msg);
      
      

      
      landing_target.pose.orientation = quat_msg;

      landing_target_set = true;
      
      tag_loc_pub.publish(landing_target);
      tag_loc_point_pub.publish(landing_target.pose.position);
    }
    
    catch (tf::TransformException ex){
      return;
      //ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
  }
  
  else
  {
    tag_visible = false;
  }

}



void Lander::Update(){

    if (target_found & landing_target_set){
        
        //tag_loc_pub.publish(landing_target);
        
        if (radio & Target_Pose_set & landing_target_set & tag_visible & Pid_Set){
          if (fabs(target_yaw) > 0.05){
            Target_Yaw.header = landing_target.header;
            Target_Yaw.twist.angular.z = target_yaw * 2.0;
            setpoint_vel_pub.publish(Target_Yaw); 
          }
          else
          {
            local_set_pose_raw_pub.publish(pid_vel_target);
          }
        }
    
    }

}


void Lander::SetRawVel(mavros_msgs::PositionTarget Pmsg){

  pid_vel_target = Pmsg;
  if (pid_vel_target.velocity.x > 0.01 || pid_vel_target.velocity.y > 0.01){
      pid_vel_target.velocity.z = 0;
  }
  Pid_Set = true;
  
}     
    
    
