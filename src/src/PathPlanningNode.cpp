#include "CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>


using std::vector;
using std::string;

using geometry_msgs::PoseStamped;
using costmap_2d::Costmap2D;
using costmap_2d::Costmap2DROS;


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");

    tf2_ros::Buffer tf(ros::Duration(10));

    costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
    //planner_costmap_ros_->pause();

    CleaningPathPlanning clr(&lcr);
    clr.GetPathInROS();
    //clr.GetBorderTrackingPathInROS();
    ros::Rate r(10);
    while(ros::ok()){
      clr.PublishCoveragePath();
      ros::spinOnce();
      r.sleep();
    }

    ros::shutdown();
    return 0;
}

