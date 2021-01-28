#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandInt.h>
#include <unistd.h>
#include <sys/reboot.h>
#include <string> 


class Listener{

  public:
    
    Listener(ros::NodeHandle nh_, int ResCH, int GainCH, int ExpCH){
    
      node_handle_ = nh_;
      FCU_restart_channel = ResCH;
      Cam_Gain_channel = GainCH;
      Cam_Exp_channel = ExpCH;
      Armed = true;
      Radio_restart_FCU = false;
      Radio_restart_OBC = false;
      Restart_Req_Sent = false;
      
      Reboot_FCU = nh_.serviceClient<mavros_msgs::CommandInt>("/mavros/cmd/command_int");
      
    
    }
    
    mavros_msgs::State current_state;
    ros::NodeHandle node_handle_;
    
    ros::ServiceClient Reboot_FCU;
    
    bool Armed;
    
    bool Radio_restart_FCU;
    bool Radio_restart_OBC;
    
    bool Restart_Req_Sent;
    
    void FcuState(mavros_msgs::State Statemsg);
    
    void Radio(mavros_msgs::RCIn RCmsg);
    
    int FCU_restart_channel;

    int Cam_Gain_channel;

    int Cam_Exp_channel;

    int GainValue;
    int NewGainValue;

    int ExpValue;
    int NewExpValue;    


};


int main(int argc, char **argv)
{

    ros::init(argc, argv,"radio_listener");
    ros::NodeHandle nh;
    
    int fcu_restart_channel_num;
    nh.param<int>("fcu_restart_channel_num",fcu_restart_channel_num,10);
    nh.getParam("fcu_restart_channel_num",fcu_restart_channel_num);

    int cam_gain_channel_num;
    nh.param<int>("cam_gain_channel_num",cam_gain_channel_num,11);
    nh.getParam("cam_gain_channel_num",cam_gain_channel_num);

    int cam_exp_channel_num;
    nh.param<int>("cam_exp_channel_num",cam_exp_channel_num,12);
    nh.getParam("cam_exp_channel_num",cam_exp_channel_num);



    
    
    Listener Listen(nh,fcu_restart_channel_num,cam_gain_channel_num,cam_exp_channel_num);
    
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
    
        if (!Listen.Armed && Listen.Radio_restart_FCU && (ros::Time::now() - last_request > ros::Duration(5.0))){
        
            std::cout<<"Restarting the FCU"<<std::endl;
        
            if(Listen.Reboot_FCU.call(Reboot_command) && Reboot_command.response.success){
                last_request = ros::Time::now();
                Listen.Restart_Req_Sent = true;
                
            }
        }
        
        if (!Listen.Armed && Listen.Radio_restart_OBC && (ros::Time::now() - last_request > ros::Duration(5.0))){
        
            std::cout<<"Restarting the OBC"<<std::endl;
            last_request = ros::Time::now();
            sync();
            std::cout<<reboot(RB_AUTOBOOT)<<std::endl;
            std::cout<<system("echo spiri-friend | sudo -S reboot now")<<std::endl;
            
        }
        
        if (Listen.NewGainValue != Listen.GainValue){
            
            std::cout<<system(("v4l2-ctl -d /dev/video0 -c gain="+ std::to_string(Listen.NewGainValue)).c_str())<<std::endl;
            std::cout<<system(("v4l2-ctl -d /dev/video1 -c gain="+ std::to_string(Listen.NewGainValue)).c_str())<<std::endl;
            Listen.GainValue = Listen.NewGainValue;
        
        }
        
        if (Listen.NewExpValue != Listen.ExpValue){
            
            std::cout<<system(("v4l2-ctl -d /dev/video0 -c exposure="+ std::to_string(Listen.NewExpValue)).c_str())<<std::endl;
            std::cout<<system(("v4l2-ctl -d /dev/video1 -c exposure="+ std::to_string(Listen.NewExpValue)).c_str())<<std::endl;
            Listen.ExpValue = Listen.NewExpValue;
        
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
    
    
    
}

void Listener::FcuState(mavros_msgs::State Statemsg){

   current_state = Statemsg;
   
   Armed = current_state.armed;

}



void Listener::Radio(mavros_msgs::RCIn RCmsg){
   std::cout<<"Radio channel value"<<std::endl;
   std::cout<<RCmsg.channels[FCU_restart_channel]<<std::endl;
   if (RCmsg.channels[FCU_restart_channel] == 982 && !Radio_restart_FCU && !Armed){
      Radio_restart_FCU = true;
      std::cout<<"FCU restart activated"<<std::endl;
      
   }
   else if (Radio_restart_FCU && RCmsg.channels[FCU_restart_channel] != 982){
      Radio_restart_FCU = false;
      
   }
   
   
   
   if (RCmsg.channels[FCU_restart_channel] == 2006 && !Radio_restart_OBC && !Armed){
      Radio_restart_OBC = true;
      std::cout<<"OBC restart activated"<<std::endl;
      
   }
   else if (Radio_restart_OBC && RCmsg.channels[FCU_restart_channel] != 2006){
      Radio_restart_OBC = false;
      
   }

   if (RCmsg.channels[Cam_Gain_channel] >= 982){

      NewGainValue = RCmsg.channels[Cam_Gain_channel] - 982;

   }
   
   if (RCmsg.channels[Cam_Exp_channel] >= 982){

      NewExpValue = (RCmsg.channels[Cam_Exp_channel] - 982)* 14;

   }
   

   
}     


