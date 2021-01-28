

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "LowPassFilter.hpp"





class RangeToRangeMeanFilter
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Components for tf::MessageFilter
  tf::TransformListener *tf_;
  message_filters::Subscriber<sensor_msgs::Range> range_sub_;
  tf::MessageFilter<sensor_msgs::Range> *tf_filter_;
  double tf_filter_tolerance_;

  LowPassFilter lpf;


  // Components for publishing
  sensor_msgs::Range msg_;
  ros::Publisher output_pub_;

  // Deprecation helpers
  ros::Timer deprecation_timer_;
  bool  using_filter_chain_deprecated_;

public:
  // Constructor
  RangeToRangeMeanFilter(double cutoff,double deltatime, ros::NodeHandle n) :
     range_sub_(nh_, "/mavros/distance_sensor/range", 50)
     {

     //filter->configure("MeanFilterDouble5");
     lpf.reconfigureFilter(cutoff,deltatime);
     //filter_chain_.configure("A");

     range_sub_.registerCallback(boost::bind(&RangeToRangeMeanFilter::callback, this, _1));
    // Advertise output
     output_pub_ = nh_.advertise<sensor_msgs::Range>("Range_filtered", 1000);


   }

  // Destructor
  ~RangeToRangeMeanFilter()
  {
    if (tf_filter_)
      delete tf_filter_;
    if (tf_)
      delete tf_;
  }
  

  // Callback
  void callback(const sensor_msgs::Range::ConstPtr& msg_in)
  {
    // Run the filter chain
    sensor_msgs::Range RangeMsg = *msg_in;
    sensor_msgs::Range FilterMsg = *msg_in;
    double RangeValue = RangeMsg.range;
    double RangeFiltered;
    RangeFiltered = lpf.update(RangeValue);
    FilterMsg.range = RangeFiltered;
    output_pub_.publish(FilterMsg);

  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_to_range_filter_chain");

  ros::NodeHandle nh;
  
  double cut_off_frequancy;
  nh.param<double>("cut_off_frequancy",cut_off_frequancy,0.5);
  nh.getParam("cut_off_frequancy",cut_off_frequancy);
  
  double delta_time;
  nh.param<double>("delta_time",delta_time,0.01);
  nh.getParam("delta_time",delta_time);
  
  RangeToRangeMeanFilter t(cut_off_frequancy,delta_time,nh);
  ros::spin();
  
  return 0;
}
