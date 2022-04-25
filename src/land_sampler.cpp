#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/WaypointList.h>

float required_height = 0.5;
float required_x = 0.0;
float required_y = 0.0;
int mav_cmd_number = 530;

mavros_msgs::State current_state;
mavros_msgs::CommandLong sampler_land_cmd;
geometry_msgs::PoseStamped current_pose, goal_pose;
bool goal=false;
bool goal_set=false;
bool offboard_enabled = false;
bool mission_cmd_match = false;
bool time_set=false;


ros::Time release_time;

int current_mission_point = -1;

int target_mission_point = -1;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose= *msg;
}

void range_sensor_cb(const sensor_msgs::Range::ConstPtr& rangeMsg){
    
    if (!offboard_enabled || !mission_cmd_match){

        return;
    }
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
        goal_pose = current_pose;
        if (!time_set){
            release_time = ros::Time::now() + ros::Duration(5.0);
            time_set = true;
        }
        
        if((release_time - ros::Time::now()).toSec() > 0 ){
            std::cout<<"release_time duration "<<release_time<<std::endl;
        }
        else{
            std::cout<<"release_time Done"<<std::endl;
            goal = true;
        }
    }
    

}

void mission_cb(const mavros_msgs::WaypointList::ConstPtr& WayMsg){
  mavros_msgs::WaypointList WayList = *WayMsg;
  mavros_msgs::Waypoint Waypoint;
  current_mission_point = WayList.current_seq;
  for (int i=0; i<WayList.waypoints.size();i++){
    Waypoint = WayList.waypoints[i];
    if (Waypoint.command==mav_cmd_number){
        target_mission_point = i;
        std::cout<<"target_mission_point "<< target_mission_point << std::endl;
    }       

  }
  if (current_mission_point >= target_mission_point){
    mission_cmd_match = true;
    std::cout<<"mission match"<<std::endl;
  }
  else{
    std::cout<<"no match"<<std::endl;
    std::cout<<WayList.waypoints[current_mission_point].command<<std::endl;

  }

  std::cout << "current mission point"<<current_mission_point<<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"land_sampler");
    ros::NodeHandle nh;
    
   
    nh.param<float>("sampler_land_alt",required_height,1.0);
    nh.getParam("sampler_land_alt",required_height);
    
    //nh.param<int>("TargetWP",target_mission_point,-1);
    //nh.getParam("TargetWP",target_mission_point);

    nh.param<int>("land_mav_cmd",mav_cmd_number,530);
    nh.getParam("land_mav_cmd",mav_cmd_number);

    
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
                ("/mavros/mission/waypoints",1,mission_cb);         
                
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::ServiceClient mav_cmd_client = nh.serviceClient<mavros_msgs::CommandLong>
            ("mavros/cmd/command");

    
    ros::Rate rate(20.0);


    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }        
            




    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::SetMode mission_set_mode;
    mission_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::SetMode pos_set_mode;
    pos_set_mode.request.custom_mode = "POSCTL";



    sampler_land_cmd.request.command = 187;
    sampler_land_cmd.request.param1 = 1.0;



    ros::Time last_request = ros::Time::now();

    while(ros::ok()){


      if(!goal){

          if (mission_cmd_match && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)) && !offboard_enabled){
              ROS_INFO("Enabling Offboard");
              
              if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                  ROS_INFO("Offboard enabled");
                  offboard_enabled = true;
              }

              last_request = ros::Time::now();

          } else {
              
              if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                  std::cout << "Vehicle not armed" << std::endl; 
                  last_request = ros::Time::now();
              }
             
              if (current_state.armed && time_set && mission_cmd_match){

                    //if(set_mode_client.call(pos_set_mode) && pos_set_mode.response.mode_sent){
                    //  ROS_INFO("POSCTL enabled");
                    //}


                    if(mav_cmd_client.call(sampler_land_cmd)){

                        std::cout << "Release cmd sent" << std::endl;
                    }

                    else
                    {
                        std::cout<<"Failed to release the sampler"<<std::endl;
                    }

              }
          }

      }
      else {
              
            std::cout<<"Mission accomplished"<<std::endl;
            mission_cmd_match = false;
            offboard_enabled =false;
            goal=false;
            goal_set=false;
            time_set = false;
            sampler_land_cmd.request.param1 = -1.0;
            mav_cmd_client.call(sampler_land_cmd);
            std::cout << "Reclose cmd sent" << std::endl;
            

            if( set_mode_client.call(mission_set_mode) && mission_set_mode.response.mode_sent){
                  ROS_INFO("Mission mode enabled");

            }
         
        }

      if(mission_cmd_match){

          local_pos_pub.publish(goal_pose);

      }
    


        ros::spinOnce();
        rate.sleep();
    
    }

    return 0;
}