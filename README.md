# peg_in_hole_project

## filter.launch
launches the following:
* Force torque sensor node, published wrench (ros package **net-ft-ros**)
* Python FT classifier, published feature labels (ros package **ft_sensor_python**)
* Virtual peg_sensor model, used to estimate expected measurements 
  (not necessary when using FT) (ros package **peg_sensor**)
* Particle filter (ros package **peg_filter**)

### peg_filter

The peg filter node does two things; 1) update the particle fitler and 2) compresses the particle filter
to a lower dimensional representation. It published the particle filter weights which are shown in Rviz and 
the compressed feautres as well.
#### publish
topics: [name,type]
* [mode_feature, std_msgs::Float64MultiArray>]
* [bel_x,visualization_msgs::Marker]
* [pfilter,sensor_msgs::PointCloud]

## server.launch
Initialised and launches the server actions. One action, plug_search, is added to the action server.

### Action plug_search (ros package Peg_hole_policy)

## peg_kuka_cmd.launch

1. call plug_search
  * startes the **plug_search** action in the action server
2. call gmm
  * starts the GMM policy via a call to a service in the plug_search action.

#### publish
* actions (velocity) to the robot controller
#### subscribe
topics: [name, type]
* [ft_classifier,std_msgs::Float32MultiArray]
* 

### Sensing
* **virtual sensor**  topic: /virtual_classifier : [std_msgs/Float64MultiArray], distance to features
* **ft sensor**       topic: /ft_classifier      : [std_msgs/Float64MultiArray]


### Useful things

* rosrun rqt_reconfigure rqt_reconfigure


# Demos
