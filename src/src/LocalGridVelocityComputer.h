#ifndef LOCALGRIDVELOCITYCOMPUTER_H
#define LOCALGRIDVELOCITYCOMPUTER_H

#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

class LocalGridVelocityComputer
{
    //typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
public:
    LocalGridVelocityComputer(costmap_2d::Costmap2DROS *costmap2dros);
    inline double GetLinearVelocity(){return LINEAR_VELOCITY;}
    inline double GetRotateVelocity(){return ROTATE_VELOCITY;}
    void SetLinearVelocity(double vel){this->LINEAR_VELOCITY =vel ;}
    void SetRotateVelocity(double rotvel){this->ROTATE_VELOCITY =rotvel;}

    void SetNextGoal(geometry_msgs::PoseStamped next_goal);
    bool IsNextGoalValid(){return checkNextPointValid();}

    ros::Duration GetTimeToRotate(bool &isleft)
    {
        if(delta_theta_> 3.14159)
        {
            isleft=false;
            return ros::Duration((6.28318 - delta_theta_) / ROTATE_VELOCITY);
        }
        else
            isleft=true;return ros::Duration(delta_theta_ / ROTATE_VELOCITY);
    }
    ros::Duration GetTimeToForword(){return ros::Duration(delta_distance_ / LINEAR_VELOCITY);}

    bool IsReachedGoal();
    bool IsReachedGoalOrientation();

private:
    //helper functions.
    void computeTransAndRotateDelta();
    void updatePose();
    bool checkNextPointValid();

    double ROTATE_VELOCITY;// deg/s, right and left have same vilocity.
    double LINEAR_VELOCITY;// m/s

    ros::Duration t_rotate_;
    ros::Duration t_linear_;
    double delta_theta_;
    double delta_distance_;

    costmap_2d::Costmap2DROS *costmap2d_ros_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped next_pose_;

    //MoveBaseActionServer *as;
};

#endif // LOCALGRIDVELOCITYCOMPUTER_H
