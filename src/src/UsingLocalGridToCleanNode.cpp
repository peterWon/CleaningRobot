#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "CleaningPathPlanner.h"
#include "tf/tf.h"
#include "LocalGridVelocityComputer.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double distance(geometry_msgs::PoseStamped point1,geometry_msgs::PoseStamped point2);

int main(int argc, char** argv){
  ros::init(argc, argv, "cleaning_navigation");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal nextGoal;

  //load the global path.
  tf2_ros::Buffer tf;
  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
  CleaningPathPlanning *pathPlanner = new CleaningPathPlanning(&lcr);
  std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS();

  //local grid move.
  LocalGridVelocityComputer *localmover=new LocalGridVelocityComputer(&lcr);
  ros::Rate rate(2);


  geometry_msgs::PoseStamped currentPose;
  //main loop
  for(int i = 0; i < fullCoverPath.size(); i++)
  {
      nextGoal.target_pose.header.frame_id = "map";
      nextGoal.target_pose.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped posestamped = fullCoverPath[i];

      if(!lcr.getRobotPose(currentPose))continue;

      if(distance(currentPose,posestamped)>pathPlanner->GetSizeOfCell()*lcr.getCostmap()->getResolution()*1.5)
      {
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
            ROS_INFO("The base failed to move forward to the next path for some reason!");
      }
      else
      {
          //call grid move in a short distance.
          localmover->SetNextGoal(posestamped);
          if(!localmover->IsNextGoalValid())continue;
          else
          {
              geometry_msgs::Twist vel;

              ros::Time ct = ros::Time::now();
              bool isleft=true;
              ros::Duration dt = localmover->GetTimeToRotate(isleft);
              vel.linear.x=0;
              vel.linear.y=0;
              vel.linear.z=0;
              vel.angular.x=0;
              vel.angular.y=0;
              if(isleft)
                vel.angular.z=localmover->GetRotateVelocity();
              else vel.angular.z= -localmover->GetRotateVelocity();

              while(ros::ok())
              {
                  if(localmover->IsReachedGoalOrientation())
                  {
                      vel.angular.z=0;
                      cmd_vel_pub.publish(vel);
                      break;
                  }
                  cmd_vel_pub.publish(vel);
                  rate.sleep();
              }
              dt = localmover->GetTimeToForword();

              vel.linear.x=localmover->GetLinearVelocity();
              while(ros::ok())
              {
                  if(localmover->IsReachedGoal())
                  {
                      vel.linear.x=0;
                      cmd_vel_pub.publish(vel);                      
                      break;
                  }
                  cmd_vel_pub.publish(vel);
                  rate.sleep();
              }

              if(localmover->IsReachedGoal())
              {
                  ROS_INFO("Oh Yes!!! the base moved a point forward in full path!");
                  pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y);
                  pathPlanner->PublishGrid();
              }
              else
              {
                  ROS_INFO("Oh NO!!!The base failed to move forward to the next path for some reason!");
              }

          }
      }
    }

  delete pathPlanner;
  return 0;
}


double distance(geometry_msgs::PoseStamped point1,geometry_msgs::PoseStamped point2)
{
    return sqrt((point1.pose.position.x-point2.pose.position.x)
                *
                (point1.pose.position.x-point2.pose.position.x)
                +
                (point1.pose.position.y-point2.pose.position.y)
                *
                (point1.pose.position.y-point2.pose.position.y));
}
