#include "CleaningPathPlanner.h"
#include <costmap_2d/cost_values.h>

CleaningPathPlanning::CleaningPathPlanning(costmap_2d::Costmap2DROS *costmap2d_ros)
{
    //temp solution.
    costmap2d_ros_ = costmap2d_ros;
    //costmap2d_ros_->updateMap();
    costmap2d_ = costmap2d_ros->getCostmap();
    //costmap2d_->saveMap("/home/wz/temp.pgm");


    ros::NodeHandle private_nh("~/cleaning_plan_nodehandle");
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("cleaning_path", 1);
    grid_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("covered_grid",1);


    string sizeOfCellString,coveredValueStr;

    SIZE_OF_CELL = 3;
    if(private_nh.searchParam("size_of_cell",sizeOfCellString))
        private_nh.param("size_of_cell",SIZE_OF_CELL,3);

    GRID_COVERED_VALUE = 0;
    if(private_nh.searchParam("grid_covered_value",coveredValueStr))
        private_nh.param("grid_covered_value",GRID_COVERED_VALUE,0);


    int sizex = costmap2d_->getSizeInCellsX();
    int sizey = costmap2d_->getSizeInCellsY();
    cout<<"The size of map is "<<sizex<<"  "<<sizey<<endl;
    resolution_ = costmap2d_->getResolution();

    srcMap_=Mat(sizey,sizex,CV_8U);
    for(int r = 0; r < sizey; r++){
      for(int c = 0; c < sizex; c++ ){
          srcMap_.at<uchar>(r,c) = costmap2d_->getCost(c,sizey-r-1);//??sizey-r-1 caution: costmap's origin is at left bottom ,while opencv's pic's origin is at left-top.
      }
    }


    initializeMats();
    initializeCoveredGrid();

    //imshow("debugMapImage",srcMap_);
    //imshow("debugCellMatImage",cellMat_);
    //waitKey(0);
    //imwrite("debug_srcmap.jpg",srcMap_);

    if(!srcMap_.empty())initialized_ = true;
    else initialized_ = false;
}

vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetPathInROS()
{
//    vector<geometry_msgs::PoseStamped> resultVec;
    if(!pathVecInROS_.empty())pathVecInROS_.clear();
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Pose pose;
    vector<cellIndex> cellvec;
    cellvec = GetPathInCV();
    /*trasnsform*/
    vector<cellIndex>::iterator iter;
    int sizey = cellMat_.rows;

    for(iter=cellvec.begin(); iter!=cellvec.end();iter++)
    {
         costmap2d_->mapToWorld((*iter).col * SIZE_OF_CELL + SIZE_OF_CELL/2 , (sizey-(*iter).row-1)*SIZE_OF_CELL + SIZE_OF_CELL/2, pose.position.x, pose.position.y);
         pose.orientation.w = cos((*iter).theta * PI / 180 / 2); //(sizey-(*iter).row-1)
         pose.orientation.x = 0;
         pose.orientation.y = 0;
         pose.orientation.z = sin((*iter).theta * PI / 180 / 2);
         posestamped.header.stamp= ros::Time::now();
         posestamped.header.frame_id = "map";
         posestamped.pose = pose;

         pathVecInROS_.push_back(posestamped);
    }
    publishPlan(pathVecInROS_);
    cout<<"The path size is "<<pathVecInROS_.size()<<endl;
    return pathVecInROS_;
}

vector<geometry_msgs::PoseStamped> CleaningPathPlanning::GetBorderTrackingPathInROS()
{
     //vector<geometry_msgs::PoseStamped> resultPathInROS;
     if(!pathVecInROS_.empty())pathVecInROS_.clear();
     geometry_msgs::PoseStamped posestamped;
     geometry_msgs::Pose pose;
     vector<cv::Point2i> resultCV;
     GetBorderTrackingPathInCV(resultCV);
     vector<bool> visitedVec;

     cv::Point2i lastPoint = resultCV[0], curPoint;
     double theta = 0.0;
     visitedVec.assign(resultCV.size(),false);
     int i,j,k,index;
     for(i = 0; i < resultCV.size(); i++)
     {
         if(!visitedVec[i])
         {
             for(k=-SIZE_OF_CELL/2;k<SIZE_OF_CELL/2;k++)
             {
                 for(j=-SIZE_OF_CELL/2;j<SIZE_OF_CELL/2;j++)
                 {
                     if(findElement(resultCV,Point2i(resultCV[i].x+j,resultCV[i].y+k),index))
                     {
                         visitedVec[index] = true;

                         curPoint = resultCV[index];
                         if(curPoint.x==lastPoint.x)
                         {
                             if(curPoint.y>lastPoint.y)theta=PI/2;
                             else theta = -PI/2;
                         }
                         else theta = atan((curPoint.y-lastPoint.x)/(curPoint.x-lastPoint.x));

                         //srcMap_'s shape is same with costmap2d_,ie.. they have same resolution and size.
                         costmap2d_->mapToWorld(resultCV[index].x,srcMap_.rows-resultCV[index].y-1,pose.position.x, pose.position.y);
                         pose.orientation.w = cos(theta * PI / 180 / 2);
                         pose.orientation.x = 0;
                         pose.orientation.y = 0;
                         pose.orientation.z = sin(theta * PI / 180 / 2);
                         posestamped.header.stamp= ros::Time::now();
                         posestamped.header.frame_id = "map";
                         posestamped.pose = pose;

                         pathVecInROS_.push_back(posestamped);
                     }
                 }
             }
         }
     }
     publishPlan(pathVecInROS_);
     return pathVecInROS_;
}


//First make border path planning in original resolution using opencv tools.Then transform the result
//in ROS format and take the robot's shape into account.
void CleaningPathPlanning::GetBorderTrackingPathInCV(vector<cv::Point2i> &resultVec)
{
    std::vector<cv::Point2i> borderPointsIndexVec;//todo:make point2i corrresponding to cellindex.
    resultVec.clear();
    if(srcMap_.empty())return;

    cv::Point2i temppoint;
    int r,c,i,j;
    for( r=0; r<srcMap_.rows; r++)
    {
        for( c=0;c<srcMap_.cols;c++)
        {
            //ROS_INFO("The value at row %d, col %d is %d",r,c,srcMap_.at<uchar>(r,c));
            if(srcMap_.at<uchar>(r,c) == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                ROS_INFO("entered once!");
                if(srcMap_.at<uchar>(r,c-1)!=srcMap_.at<uchar>(r,c+1)
                        && (srcMap_.at<uchar>(r,c-1) == costmap_2d::FREE_SPACE
                           || srcMap_.at<uchar>(r,c+1)== costmap_2d::FREE_SPACE))
                {
                    temppoint.x = c;
                    temppoint.y = r;
                    borderPointsIndexVec.push_back(temppoint);
                }
                else if(srcMap_.at<uchar>(r-1,c)!=srcMap_.at<uchar>(r+1,c)
                        &&(srcMap_.at<uchar>(r-1,c) == costmap_2d::FREE_SPACE
                           || srcMap_.at<uchar>(r+1,c)== costmap_2d::FREE_SPACE) )
                {
                    temppoint.x = c;
                    temppoint.y = r;
                    borderPointsIndexVec.push_back(temppoint);
                }
            }
        }
    }

    std::cout<<"The border path length is:"<<borderPointsIndexVec.size()<<std::endl;
    std::vector<bool> visitedVec;
    if(borderPointsIndexVec.empty())return;

    int minDistIndex=-1,loopCounter=0;
    double dist,minDist;
    visitedVec.assign(borderPointsIndexVec.size(),false);

    Point2i curPoint = borderPointsIndexVec[0];
    visitedVec[0]=true;

    while(loopCounter < borderPointsIndexVec.size())
    {
        minDist = 1e6;
        resultVec.push_back(curPoint);
        for(j=1;j<borderPointsIndexVec.size();j++)
        {
            if(!visitedVec[j])
            {
                dist = distance(curPoint,borderPointsIndexVec[j]);
                if(dist < minDist)
                {
                    minDist=dist;
                    minDistIndex=j;
                }
            }
        }
        curPoint = borderPointsIndexVec[minDistIndex];
        visitedVec[minDistIndex] = true;
        loopCounter++;
    }

    //Mat resultmat=Mat(srcMap_.rows,srcMap_.cols, CV_8UC3);;
    //writeResult(resultmat,resultVec);
}

void CleaningPathPlanning::SetCoveredGrid(double wx, double wy)
{
    unsigned int mx,my,index;
    bool isok = costmap2d_->worldToMap(wx,wy,mx,my);
    if(!isok)
    {
        return;
    }

    for(int dx = -SIZE_OF_CELL/2; dx < SIZE_OF_CELL/2+1; dx++)
     {
        for(int dy =-SIZE_OF_CELL/2; dy<SIZE_OF_CELL/2+1; dy++)
        {
            index = costmap2d_->getIndex(mx+dx,my+dy);
            covered_path_grid_.data[index] = GRID_COVERED_VALUE;
        }
    }
}

void CleaningPathPlanning::PublishGrid()
{
    if(!initialized_)initializeCoveredGrid();
    grid_pub_.publish(covered_path_grid_);
}

vector<cellIndex> CleaningPathPlanning::GetPathInCV()
{
    mainPlanningLoop();
    return this->pathVec_;
}

void CleaningPathPlanning::PublishCoveragePath(){
  publishPlan(this->pathVecInROS_);
}

void CleaningPathPlanning::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
{
    if (!initialized_) {
        ROS_ERROR(
           "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool CleaningPathPlanning::cellContainsPoint(Point2i pt, cellIndex cell)
{
    return pt.y>cell.row*SIZE_OF_CELL && pt.y<(cell.row+1)*SIZE_OF_CELL
            && pt.x>cell.col*SIZE_OF_CELL && pt.x<(cell.col+1)*SIZE_OF_CELL;
}

bool CleaningPathPlanning::initializeMats()
{
    //initialize the member variables.
    if(srcMap_.empty())return false;
    getCellMatAndFreeSpace(srcMap_,cellMat_,freeSpaceVec_);

    neuralizedMat_ = Mat(cellMat_.rows,cellMat_.cols,CV_32F);
    initializeNeuralMat(cellMat_,neuralizedMat_);
    return true;
}

void CleaningPathPlanning::getCellMatAndFreeSpace(Mat srcImg, Mat &cellMat,vector<cellIndex> &freeSpaceVec)
{
    cellMat = Mat(srcImg.rows / SIZE_OF_CELL,srcImg.cols / SIZE_OF_CELL, srcImg.type());

    freeSpaceVec.clear();
    bool isFree = true;
    int r = 0,c = 0, i = 0,j = 0;
    for (r = 0;r < cellMat.rows; r++ )
    {
        for(c = 0 ; c<cellMat.cols; c++)
        {
            isFree = true;
            for(i = 0; i<SIZE_OF_CELL; i++)
            {
                for(j = 0; j < SIZE_OF_CELL; j++)
                {
                    if(srcImg.at<uchar>(r*SIZE_OF_CELL+i,c*SIZE_OF_CELL+j) != costmap_2d::FREE_SPACE)
                    {
                        isFree=false;
                        i = SIZE_OF_CELL;
                        break;
                    }
                }
            }
            if(isFree)
            {
                cellIndex ci;
                ci.row = r;
                ci.col = c;
                ci.theta = 0;
                freeSpaceVec.push_back(ci);
                cellMat.at<uchar>(r,c) = costmap_2d::FREE_SPACE;//255
            }
            else{cellMat.at<uchar>(r,c) = costmap_2d::LETHAL_OBSTACLE;}//253
        }
    }
    cout<<"freespace size:"<< freeSpaceVec.size()<<endl;
    //imwrite("cellMat.jpg",cellMat);
    return;
}

void CleaningPathPlanning::initializeNeuralMat(Mat cellMat, Mat neuralizedMat)
{
    int i = 0,j = 0;
    for(i = 0; i < neuralizedMat.rows; i++)
    {
        for(j = 0;j< neuralizedMat.cols; j++ )
        {
            if(cellMat.at<uchar>(i,j) == costmap_2d::LETHAL_OBSTACLE) neuralizedMat.at<float>(i,j) = -1.0;
            else neuralizedMat.at<float>(i,j) = 1.0 / j;
        }
    }
    return;
}


void CleaningPathPlanning::writeResult(Mat resultmat,vector<cellIndex> pathVec)
{
    int i = 0,j = 0;
    Point initpoint = Point(pathVec[0].col*SIZE_OF_CELL+SIZE_OF_CELL/2,pathVec[0].row*SIZE_OF_CELL+SIZE_OF_CELL/2);
    for(i = 1; i<pathVec.size();i++)
    {
        Point cupoint = Point(pathVec[i].col*SIZE_OF_CELL+SIZE_OF_CELL/2,pathVec[i].row*SIZE_OF_CELL+SIZE_OF_CELL/2);
        if(sqrt((initpoint.x-cupoint.x)*(initpoint.x-cupoint.x)+(initpoint.y-cupoint.y)*(initpoint.y-cupoint.y))>2)
        {
            line(resultmat,initpoint,cupoint,Scalar(0,255,0),0.3,8);
        }
        else line(resultmat,initpoint,cupoint,Scalar(0,0,255),0.5);
        initpoint = cupoint;
        cout << "The point of step "<<i<<" is: "<<pathVec[i].row<<" "<<pathVec[i].col<<endl;
        /*resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[0] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[1] = 0;
        resultMat.at<Vec3b>(pathVec[i].row*SIZE_OF_CELL,pathVec[i].col*SIZE_OF_CELL)[2] = 255;*/
    }
//    imshow("resultMat",resultmat);
//    waitKey(0);
    imwrite("reaultMat.jpg",resultmat);
}

void CleaningPathPlanning::writeResult(Mat resultmat,vector<cv::Point2i> pathVec)
{
    int i = 0,j = 0;
    Point initpoint = pathVec[0];
    for(i = 1; i<pathVec.size();i++)
    {
        Point cupoint = pathVec[i];
        line(resultmat,initpoint,cupoint,Scalar(0,0,255),0.5);
        initpoint = cupoint;
        //std::cout<<"X: "<<cupoint.x<<","<<"Y:"<<cupoint<<std::endl;

    }
//    imshow("resultMat",resultmat);
//    waitKey(0);
}

void CleaningPathPlanning::mainPlanningLoop()
{
    cellIndex initPoint,nextPoint, currentPoint;
//    initPoint.row = cellMat_.rows/2; //initPoint to be made interface.
//    initPoint.col = cellMat_.cols/2;
    initPoint.theta = 90;
    bool isok = costmap2d_ros_->getRobotPose(initPose_);
    if(!isok)
    {
        ROS_INFO("Failed to get robot location! Please check where goes wrong!");
        return;
    }
    //initPoint.row = initPose_.getOrigin().y()
    unsigned int mx,my;
    double wx = initPose_.getOrigin().x();
    double wy = initPose_.getOrigin().y();
    //geometry_msgs::PoseStamped current_position;
    //tf::poseStampedTFToMsg(global_pose, current_position);

    bool getmapcoor = costmap2d_->worldToMap(wx,wy,mx,my);
    if(!getmapcoor)
    {
        ROS_INFO("Failed to get robot location in map! Please check where goes wrong!");
        return;
    }
    initPoint.row = cellMat_.rows - my/SIZE_OF_CELL - 1;
    initPoint.col = mx/SIZE_OF_CELL;


    currentPoint = initPoint;
    pathVec_.clear();
    pathVec_.push_back(initPoint);

    float initTheta = initPoint.theta; //initial orientation
    const float c_0 = 0.001;
    float e = 0.0, v = 0.0, deltaTheta = 0.0, lasttheta = initTheta, PI = 3.14159;
    vector<float> thetaVec = {0, 45,90,135,180,225,270,315};

    //the main planning loop
    while(freeSpaceVec_.size()>0)
    {
        //erase current point from free space first.
        vector<cellIndex>::iterator it;
        for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();)
        {
            if((*it).row==nextPoint.row && (*it).col==nextPoint.col)
            {it = freeSpaceVec_.erase(it);continue;}
            it++;
        }

        //compute neiborhood's activities
        int maxIndex = 0;
        float max_v = -3;
        neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col) = -2.0;
        for(int id = 0; id < 8; id++)
        {
            deltaTheta = max(thetaVec[id],lasttheta)-min(thetaVec[id],lasttheta);
            if(deltaTheta>180) deltaTheta=360-deltaTheta;
            e = 1 - abs(deltaTheta) / 180;
            switch (id)
            {
                case 0:
                    if(currentPoint.col==neuralizedMat_.cols-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col+1) + c_0 * e;
                    break;
                case 1:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col+1) + c_0 * e;
                    break;
                case 2:
                if(currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col) + c_0 * e;
                    break;
                case 3:
                if(currentPoint.col== 0 || currentPoint.row == 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row-1 ,currentPoint.col-1) + c_0 * e;
                    break;
                case 4:
                if(currentPoint.col== 0){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row ,currentPoint.col-1) + c_0 * e;
                    break;
                case 5:
                if(currentPoint.col== 0 || currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col-1) + c_0 * e;
                    break;
                case 6:
                if(currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col) + c_0 * e;
                    break;
                case 7:
                if(currentPoint.col==neuralizedMat_.cols-1 || currentPoint.row == neuralizedMat_.rows-1){v=-2;break;}
                    v = neuralizedMat_.at<float>(currentPoint.row+1 ,currentPoint.col+1) + c_0 * e;
                    break;
                default:
                    break;
            }
            if(v > max_v)
            {
                max_v = v;
                maxIndex=id;
            }
        }


        if(max_v <= 0)
        {
            float dist = 0.0, min_dist = 100000;
            //vector<cellIndex>::iterator min_iter;
            int ii=0, min_index=-1;
            for(it=freeSpaceVec_.begin();it!=freeSpaceVec_.end();it++)
            {
                if(neuralizedMat_.at<float>((*it).row,(*it).col) > 0)
                {
                    dist = sqrt((currentPoint.row-(*it).row)*(currentPoint.row-(*it).row)+(currentPoint.col-(*it).col)*(currentPoint.col-(*it).col));
                    if(dist < min_dist)
                    {
                        min_dist = dist;
                        min_index = ii;
                    }
                }
                ii++;
            }
            if(min_dist==0 || min_index == -1)
            {break;}
            else
            {
                cout << "next point index: "<<min_index<< endl;
                cout << "distance: "<<min_dist << endl;
                nextPoint = freeSpaceVec_[min_index];
                currentPoint = nextPoint;
                pathVec_.push_back(nextPoint);

                continue;
            }
        }

        //next point.
        switch (maxIndex)
        {
        case 0:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col+1;
            break;
        case 1:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col+1;
            break;
        case 2:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col;
            break;
        case 3:
            nextPoint.row = currentPoint.row-1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 4:
            nextPoint.row = currentPoint.row;
            nextPoint.col = currentPoint.col-1;
            break;
        case 5:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col-1;
            break;
        case 6:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col;
            break;
        case 7:
            nextPoint.row = currentPoint.row+1;
            nextPoint.col = currentPoint.col+1;
            break;
        default:
            break;
        }
        nextPoint.theta = thetaVec[maxIndex];
        currentPoint = nextPoint;
        pathVec_.push_back(nextPoint);
    }

    Mat resultMat = Mat(srcMap_.rows,srcMap_.cols, CV_8UC3);
    writeResult(resultMat,pathVec_);
}

double CleaningPathPlanning::distance(Point2i pta, Point2i ptb)
{
    return sqrt((pta.x-ptb.x)*(pta.x-ptb.x)+(pta.y-ptb.y)*(pta.y-ptb.y));
}

bool CleaningPathPlanning::findElement(vector<Point2i> pointsVec, Point2i pt, int &index)
{
    for(int i = 0;i<pointsVec.size();i++)
    {
        if(pointsVec[i].x == pt.x && pointsVec[i].y == pt.y)
           { index = i;
        return true;}
    }
    index = -1;
    return false;
}



bool CleaningPathPlanning::initializeCoveredGrid()
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap2d_->getMutex()));
    double resolution = costmap2d_->getResolution();

    covered_path_grid_.header.frame_id = "map";
    covered_path_grid_.header.stamp = ros::Time::now();
    covered_path_grid_.info.resolution = resolution;

    covered_path_grid_.info.width = costmap2d_->getSizeInCellsX();
    covered_path_grid_.info.height = costmap2d_->getSizeInCellsY();

    double wx, wy;
    costmap2d_->mapToWorld(0, 0, wx, wy);
    covered_path_grid_.info.origin.position.x = wx - resolution / 2;
    covered_path_grid_.info.origin.position.y = wy - resolution / 2;
    covered_path_grid_.info.origin.position.z = 0.0;
    covered_path_grid_.info.origin.orientation.w = 1.0;

    covered_path_grid_.data.resize(covered_path_grid_.info.width * covered_path_grid_.info.height);

    unsigned char* data = costmap2d_->getCharMap();
    for (unsigned int i = 0; i < covered_path_grid_.data.size(); i++)
    {
        /*if(data[i]==costmap_2d::FREE_SPACE)
            covered_path_grid_.data[i] = costmap_2d::FREE_SPACE;
        else
            covered_path_grid_.data[i] = 0;*/
        covered_path_grid_.data[i] = data[i];
    }
    return true;
}
