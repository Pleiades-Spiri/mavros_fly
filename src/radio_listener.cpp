#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandInt.h>

class Listener{

  public:
    
    Listener(ros::NodeHandle nh_, int ResCH){
    
      node_handle_ = nh_;
      FCU_restart_channel = ResCH;
      Armed = true;
      Radio_restart = false;
      Restart_Req_Sent = false;
      
      Reboot_FCU = nh_.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command_int");
      
    
    }
    
    mavros_msgs::State current_state;
    ros::NodeHandle node_handle_;
    
    ros::ServiceClient Reboot_FCU;
    
    bool Armed;
    
    bool Radio_restart;
    
    bool Restart_Req_Sent;
    
    void FcuState(mavros_msgs::State Statemsg);
    
    void Radio(mavros_msgs::RCIn RCmsg);
    
    int FCU_restart_channel;
    


};


int main(int argc, char **argv)
{

    ros::init(argc, argv,"radio_listener");
    ros::NodeHandle nh;
    
    int fcu_restart_channel_num;
    nh.param<int>("fcu_restart_channel_num",fcu_restart_channel_num,10);
    nh.getParam("fcu_restart_channel_num",fcu_restart_channel_num);
    
    
    Listener Listen(nh,fcu_restart_channel_num);
    
    ros::Subscriber radio_sub = nh.subscribe("/mavros/rc/in", 10, &Listener::Radio, &Listen);
    

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, &Listener::FcuState, &Listen);

   
    

    ros::Rate rate(20.0);
    
    while(ros::ok() && !Listen.current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::CommandInt Reboot_command;
    Reboot_command.request.command = 246;
    Reboot_command.request.param1 = 1;
    
    ros::Time last_request = ros::Time::now();
    
    while(ros::ok()){
    
        if (!Listen.Armed && Listen.Radio_restart && (ros::Time::now() - last_request > ros::Duration(5.0))){
        
            std::cout<<"Restarting the FCU"<<std::endl;
        
            if(Listen.Reboot_FCU.call(Reboot_command) && Reboot_command.response.success){
                last_request = ros::Time::now();
                Listen.Restart_Req_Sent = true;
                
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    
    
}

void Listener::FcuState(mavros_msgs::State Statemsg){

   current_state = Statemsg;
   
   Armed = current_state.armed;

}



void Listener::Radio(mavros_msgs::RCIn RCmsg){
   std::cout<<"Radio channel value"<<std::endl;
   std::cout<<RCmsg.channels[FCU_restart_channel]<<std::endl;
   if (RCmsg.channels[FCU_restart_channel] == 982 && !Radio_restart && !Armed){
      Radio_restart = true;
      std::cout<<"Radio restaet activated"<<std::endl;
      
   }
   else if (Radio_restart && RCmsg.channels[FCU_restart_channel] != 982){
      Radio_restart = false;
      
   }
   
}     


