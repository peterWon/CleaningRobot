#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "CleaningPathPlanner.h"
#include "tf/tf.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "cleaning_using_movebase");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal nextGoal;

  //load the global path.
  tf::TransformListener tl_listener(ros::Duration(10));
  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tl_listener);
  CleaningPathPlanning *pathPlanner = new CleaningPathPlanning(&lcr);
  std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS();
  int beginNum = fullCoverPath.size();
  std::vector<geometry_msgs::PoseStamped> borderTrackingPath = pathPlanner->GetBorderTrackingPathInROS();
  for(int i = 0;i<borderTrackingPath.size();i++)
  {
      fullCoverPath.push_back(borderTrackingPath[i]);
  }
  //main loop
  for(int i = beginNum+1; i < fullCoverPath.size(); i++)
  {
      nextGoal.target_pose.header.frame_id = "map";
      nextGoal.target_pose.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped posestamped = fullCoverPath[i];

      //call move base to plan a long distance.
      nextGoal.target_pose.pose.position.x = posestamped.pose.position.x;
      nextGoal.target_pose.pose.position.y = posestamped.pose.position.y;
      nextGoal.target_pose.pose.position.z = 0;
      nextGoal.target_pose.pose.orientation.w = posestamped.pose.orientation.w;
      nextGoal.target_pose.pose.orientation.x = posestamped.pose.orientation.x;
      nextGoal.target_pose.pose.orientation.y = posestamped.pose.orientation.y;
      nextGoal.target_pose.pose.orientation.z = posestamped.pose.orientation.z;

      ROS_INFO("Sending next goal!");
      ac.sendGoal(nextGoal);
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Hooray, the base moved a point forward in full path!");
          pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y);
          pathPlanner->PublishGrid();
      }
      else
      {
          ROS_INFO("The base failed to move forward to the next path for some reason!");
          continue;
      }
    }

  delete pathPlanner;
  return 0;
}

/*
void reconfigureGlobalPath()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "/move_base";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}

void reconfigureLocalPath()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "kurtana_pitch_joint";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}

void reconfigureBorderTracking()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "kurtana_pitch_joint";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}
*/
