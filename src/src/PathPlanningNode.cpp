#include "CleaningPathPlanner.h"
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;


int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");

    tf::TransformListener tf(ros::Duration(10));

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

