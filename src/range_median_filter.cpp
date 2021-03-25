#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "message_filters/subscriber.h"



class RangeToRangeMedianFilter
{

  protected:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  
  sensor_msgs::Range msg_;
  ros::Publisher output_pub_;
  ros::Subscriber range_sub_;

  std::vector<double> data_in;
  
  public:
  // Constructor
  RangeToRangeMedianFilter(int size,ros::NodeHandle n)
     {
       
       //filter->configure(rows,"MeanFilterDouble1");

       data_in = std::vector<double> (size,0.0);


       range_sub_ = nh_.subscribe("/mavros/distance_sensor/range", 100, &RangeToRangeMedianFilter::callback,this);
      // Advertise output
       output_pub_ = nh_.advertise<sensor_msgs::Range>("Range_filtered", 10);


    }
    
    void callback(const sensor_msgs::Range::ConstPtr& msg_in)
    {
      // Run the filter chain
      std::cout<< "inside callback "<<data_in.size()<<std::endl;
      for (std::vector<double>::const_iterator i = data_in.begin(); i != data_in.end(); ++i)
          std::cout << *i << ' ';
      std::cout<<std::endl;
      sensor_msgs::Range RangeMsg = *msg_in;
      sensor_msgs::Range FilterMsg = *msg_in;
      double RangeValue = RangeMsg.range;
      double RangeFiltered;
      
      
      data_in.erase(data_in.begin());
      for (std::vector<double>::const_iterator i = data_in.begin(); i != data_in.end(); ++i)
          std::cout << *i << ' ';
      std::cout<<std::endl;
       
      data_in.push_back(RangeValue);
      for (std::vector<double>::const_iterator i = data_in.begin(); i != data_in.end(); ++i)
          std::cout << *i << ' ';
      std::cout<<std::endl;
      
      size_t size = data_in.size();
      std::vector<double> Sorted = data_in;
      std::sort(Sorted.begin(),Sorted.end());

      if (data_in.size() % 2 == 0) {
        RangeFiltered = (Sorted[size / 2 - 1] + Sorted[size / 2]) / 2 ;
        
      }
      else {
       RangeFiltered = Sorted[size / 2] ;
      }
      
      std::cout<< RangeFiltered << std::endl;
      
      FilterMsg.range = RangeFiltered;
      output_pub_.publish(FilterMsg);

    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_to_range_median_filter");

  ros::NodeHandle nh;
  
  double FilterSize;
  nh.param<double>("FilterSize",FilterSize,5);
  nh.getParam("FilterSize",FilterSize);
  
  
  RangeToRangeMedianFilter filter(FilterSize,nh);
  ros::spin();
  
  return 0;
}









