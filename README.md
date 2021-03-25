# Mavros Fly nodes

Nodes for fly controlling using mavros

# Build

Clone the repo to your workspace and `catkin build mavros_fly`

## Nodes

 - **mavros_fly_range_to_range_filter_chain**
	 - Node for filtering the rangefinder's signal on topic `/mavros/distance_sensor/range`, through applying a lowpass filter, the output publised topic is `Range_filtered`
	 
 - **mavros_fly_descent_using_rangefinder**
	 - Node for descending spiri's altitude using a range topic. The node uses a ros param server to load the subscribed to topic, required altitude and which waypoint on the flight mission to start the descent. 
	 - `roslaunch mavros_fly sampling_using_rangefinder.launch WP:=0 Alt:=1 RangeTopic:=/Range_filtered` will start the descending at waypoint 0 i.e. immediately, to an altitude of 1.0 m, with a subscribtion to topic /Range_filtered. Default waypoint is 0 and altitude is 1.0 m.
	 - The node keeps Spiri at  +/- 0.1 m  the set altitude, after a total of accumalated time of 4 sec after that the flight mode will switch back to "Mission" mode.
