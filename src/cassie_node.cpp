#include <string>
#include <iostream>
#include <ros/ros.h>

#include "bkioctomap.h"
#include "markerarray_pub.h"
#include "cassie_util.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cassie_node");
    ros::NodeHandle nh("~");

    std::string map_topic("/occupied_cells_vis_array");
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    float prior = 1.0f;
    float var_thresh = 1.0f;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double resolution = 0.1;
    int num_class = 14;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    double max_range = -1;
    
    nh.param<std::string>("topic", map_topic, map_topic);
    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<float>("prior", prior, prior);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("num_class", num_class, num_class);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<double>("max_range", max_range, max_range);
    
    ROS_INFO_STREAM("Parameters:" << std::endl <<
      "topic: " << map_topic << std::endl <<
      "block_depth: " << block_depth << std::endl <<
	    "sf2: " << sf2 << std::endl <<
      "ell: " << ell << std::endl <<
      "prior: " << prior << std::endl <<
      "var_thresh: " << var_thresh << std::endl <<
      "free_thresh: " << free_thresh << std::endl <<
      "occupied_thresh: " << occupied_thresh << std::endl <<
      "resolution: " << resolution << std::endl <<
      "num_class: " << num_class << std::endl <<
      "free_resolution: " << free_resolution << std::endl <<
      "ds_resolution: " << ds_resolution << std::endl <<
      "max_range: " << max_range
      );

    
    ///////// Build Map /////////////////////
    CassieData cassie_data(nh, resolution, block_depth, num_class,
                           sf2, ell, prior,
                           var_thresh, free_thresh, occupied_thresh,
                           ds_resolution, free_resolution, max_range,
                           map_topic);
    ros::Subscriber sub = nh.subscribe("/labeled_pointcloud", 5000, &CassieData::PointCloudCallback, &cassie_data);

    ros::spin();

    return 0;
}
