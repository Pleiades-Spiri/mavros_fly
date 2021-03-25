#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/WaypointList.h>

float required_height = 1.0;
float required_x = 0.0;
float required_y = 0.0;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose, goal_pose;
bool goal=false;
bool goal_set=false;
bool offboard_enabled = false;

float Sampling_time = -1;

int current_mission_point = -1;

int target_mission_point = -1;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose= *msg;
}

void range_sensor_cb(const sensor_msgs::Range::ConstPtr& rangeMsg){
    
    ros::Time Start = ros::Time::now();
    sensor_msgs::Range range_msg = *rangeMsg;
    if (!goal){
        std::cout<<"Range Value :"<<range_msg.range<<std::endl;
    }    

    if(range_msg.range>required_height){
        goal_pose = current_pose;
        goal_pose.pose.position.z = current_pose.pose.position.z - 0.1;
    
    }
    else if (required_height-range_msg.range>0.5){
        goal_pose = current_pose;
        goal_pose.pose.position.z = current_pose.pose.position.z + 0.1;
    
    }
    else{
        ros::Duration(0.2).sleep();
        if (Sampling_time == -1){
            Sampling_time = 0;
        }
        
        if(Sampling_time < 4){
            float Duration = (ros::Time::now() - Start).toSec();
            std::cout<<"Sampling duration "<<Sampling_time<<std::endl;
            Sampling_time = Sampling_time + Duration;    
        }
        else{
            std::cout<<"Sampling Done"<<std::endl;
            goal = true;
        }
    }
    

}

void mission_cb(const mavros_msgs::WaypointList::ConstPtr& WayMsg){
  mavros_msgs::WaypointList WayList = *WayMsg;
  current_mission_point = WayList.current_seq;
  std::cout << "current mission point"<<current_mission_point<<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"descent_using_rangefinder");
    ros::NodeHandle nh;
    
   
    nh.param<float>("alt",required_height,1.0);
    nh.getParam("alt",required_height);
    
    nh.param<int>("TargetWP",target_mission_point,-1);
    nh.getParam("TargetWP",target_mission_point);
    
    //required_height = std::stof(Alt_str,&sz);
    
    std::cout<<"Lowering to the altittude of "<<required_height<<std::endl;
    std::cout<<"At WP: "<<target_mission_point<<std::endl;
    
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
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>
                ("/mavros/distance_sensor/range", 10, range_sensor_cb);
                
    ros::Subscriber mission_sub = nh.subscribe<mavros_msgs::WaypointList>
                ("/mavros/mission/waypoints",10,mission_cb);            
                
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
    /*goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal_pose.pose.position.z = 0;

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(goal_pose);
        ros::spinOnce();
        rate.sleep();
    }*/

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    //mavros_msgs::CommandBool arm_cmd;
    //arm_cmd.request.value = true;

    

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){


      if(!goal){

          if (current_mission_point == target_mission_point && current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(5.0)) && !offboard_enabled){
              ROS_INFO("Enabling Offboard");
              if( set_mode_client.call(offb_set_mode) &&
                  offb_set_mode.response.mode_sent){
                  ROS_INFO("Offboard enabled");
                  offboard_enabled = true;
              }
              last_request = ros::Time::now();
          } else {
              if( !current_state.armed &&
                  (ros::Time::now() - last_request > ros::Duration(5.0))){
                  std::cout << "Vehicle not armed" << std::endl; 
                  last_request = ros::Time::now();
              }
          }

          if (current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
              
              //local_pos_pub.publish(goal_pose);

          }
      }
      else {
              
          
            std::cout<<"Mission accomplished"<<std::endl;
            offb_set_mode.request.custom_mode = "AUTO.MISSION";
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                  ROS_INFO("Mission mode enabled");
                  return 1;
            }
          

         
      }


        local_pos_pub.publish(goal_pose);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
