#include "LocalGridVelocityComputer.h"
#include <costmap_2d/cost_values.h>

LocalGridVelocityComputer::LocalGridVelocityComputer(costmap_2d::Costmap2DROS *costmap2dros)
{
    this->costmap2d_ros_ = costmap2dros;
    ROTATE_VELOCITY = 0.025;
    LINEAR_VELOCITY = 0.025;
}

void LocalGridVelocityComputer::SetNextGoal(geometry_msgs::PoseStamped next_goal)
{
    next_pose_ = next_goal;
    updatePose();
    computeTransAndRotateDelta();
}

bool LocalGridVelocityComputer::IsReachedGoal()
{
    updatePose();
    computeTransAndRotateDelta();
    if(delta_distance_ < costmap2d_ros_->getCostmap()->getResolution()*4)
        return true;
    return false;
}

bool LocalGridVelocityComputer::IsReachedGoalOrientation()
{
    updatePose();
    computeTransAndRotateDelta();
    if(delta_theta_<0.02)return true;
    return false;

}

void LocalGridVelocityComputer::computeTransAndRotateDelta()
{
    double delta_theta = next_pose_.pose.orientation.w - current_pose_.pose.orientation.w;
    double deltaTranslation = sqrt((next_pose_.pose.position.x - current_pose_.pose.position.x)
                                  *
                                  (next_pose_.pose.position.x - current_pose_.pose.position.x)
                                  +
                                  (next_pose_.pose.position.y - current_pose_.pose.position.y)
                                  *
                                  (next_pose_.pose.position.y - current_pose_.pose.position.y));
    delta_theta_ = delta_theta;
    delta_distance_ = deltaTranslation;
}

void LocalGridVelocityComputer::updatePose()
{
    tf::Stamped<tf::Pose> pose;
    if(costmap2d_ros_->getRobotPose(pose))
        tf::poseStampedTFToMsg(pose, current_pose_);
}

bool LocalGridVelocityComputer::checkNextPointValid()
{
    costmap2d_ros_->updateMap();
    unsigned int mx,my;
    costmap_2d::Costmap2D *costmap2d = costmap2d_ros_->getCostmap();

    std::vector<geometry_msgs::Point> footprints = costmap2d_ros_->getRobotFootprint();
    std::vector<geometry_msgs::Point>::iterator iter;
    for(iter= footprints.begin();iter!=footprints.end();iter++)
    {
        if(!costmap2d->worldToMap( (*iter).x + delta_distance_ * cos(delta_theta_),
                                   (*iter).y+ delta_distance_ * sin(delta_theta_), mx, my))return false;
        if(costmap2d->getCost(mx,my)!=costmap_2d::FREE_SPACE)return false; //FREE_SPACE=0
    }
    return true;
}

