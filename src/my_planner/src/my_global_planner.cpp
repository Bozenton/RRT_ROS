#include <my_global_planner/my_global_planner.h>

#include <pluginlib/class_list_macros.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(my_global_planner::MyPlannerROS, nav_core::BaseGlobalPlanner)

namespace my_global_planner{


MyPlannerROS::MyPlannerROS()
: costmap_(NULL), initialized_(false){}

/**
 * @brief Construct a new MyPlannerROS object
 * 
 * @param name 
 * @param costmap 
 * @param frame_id 
 */
MyPlannerROS::MyPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
: costmap_(NULL), initialized_(false){
    //initialize the planner
    initialize(name, costmap, frame_id);
}

MyPlannerROS::~MyPlannerROS() {
    for(auto i:rrtTree)
        delete i;
    for(auto i:rrtConTree)
        delete i;
    for(auto i:rrtStarTree)
        delete i;
    
    delete rrtStartNode;
    delete rrtConStartNode;
    delete rrtStarStartNode;

    delete rrtGoalNode;
    delete rrtConGoalNode;
    delete rrtStarGoalNode;

    delete splineSmoother;
}

void MyPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void MyPlannerROS::initialize(const std::string& name, costmap_2d::Costmap2D* costmap, std::string frame_id){
    if(!initialized_)
    {
        global_frame_id_ = frame_id;
        mapInit(costmap);
        obstacleThreshold = 128;

        ros::NodeHandle prv_nd("~/"+name);

        splineSmoother = new PathSplineSmoother();

        // 设置RRT参数
        whichAlgorithm = 1;
        maxIteration = 2000;
        targetNum = 30;
        stepLen = 0.15;
        randProbility = 0.3;
        goalProbility = 1-randProbility;
        withinDis = 0.15;
        checkStepLen = 0.01;
        smoothOrNot = true;
        rrtStarRange = 8;

        planPub = prv_nd.advertise<nav_msgs::Path>("plan", 1);
        treePub = prv_nd.advertise<sensor_msgs::PointCloud>("rrt_tree", 1);
        samplePub = prv_nd.advertise<sensor_msgs::PointCloud>("rrt_sample", 1);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        initialized_ = true;
        ROS_INFO("Initialized the rrt. ");
    }
    else{
        ROS_WARN("Initialization failed or it has already been initialized");
    }
}

void MyPlannerROS::mapInit(costmap_2d::Costmap2D * costmap)
{
    costmap_ = costmap;

    // 获得地图参数
    mapOriginX = costmap_->getOriginX();
    mapOriginY = costmap_->getOriginY();
    mapSizeInCellsX = costmap_->getSizeInCellsX();
    mapSizeInCellsY = costmap_->getSizeInCellsY();
    mapAreaInCells = mapSizeInCellsX * mapSizeInCellsY;
    mapResolution = costmap_->getResolution();

    ROS_INFO(" Map information: ");
    ROS_INFO("mapSizeInCellsX = %d, mapSizeInCellsY = %d", mapSizeInCellsX, mapSizeInCellsX);
    ROS_INFO("mapResolution = %f", mapResolution);

    // 计算可行域的个数
    freeSpaceCnt = 0;
    for(int i=0; i<mapSizeInCellsX; i++)
    {
        for(int j=0; j<mapSizeInCellsY; j++)
        {
            unsigned char cost; 
            cost = costmap_->getCost(i, j);
            if(cost < obstacleThreshold)
                freeSpaceCnt ++;
        }
    }
    ROS_INFO("Map initialize done. ");
}

/// This function can not be used from Map3D because it expects an integer cell value!
/// it is preferred to leave the function as it is in Map3D (and not change it to doubles)
void MyPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = mapOriginX + mx * mapResolution;
    wy = mapOriginY + my * mapResolution;
}

bool MyPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = global_frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {

        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }
    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {

        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }
    
    

    ROS_INFO("Begin to plan now. ");

    if(getPlan(start, goal, plan))
    {
        ROS_INFO("Get plan with len = %d", int(plan.size()));
        publishPlan(plan);
        return true;
    }
    else
    {
        ROS_WARN("NO PATH FOUND FROM THE RRT PLANNER");
        return false;
    }

}


void MyPlannerROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool MyPlannerROS::getPlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
	std::vector<geometry_msgs::PoseStamped>& plan) {
    
    // ------------------------------------------------------
    // 坐标变换
    // ------------------------------------------------------
    ROS_INFO("Coordinate transform");

    wx_start = start.pose.position.x;
    wy_start = start.pose.position.y;
    // 将实际的位置坐标转化为地图上的位置坐标
    if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)){
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    wx_goal = goal.pose.position.x;
    wy_goal = goal.pose.position.y;
    if(!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)){
        ROS_WARN("The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    
    ROS_INFO("START x = %d, y = %d", mx_start, my_start);
    ROS_INFO("GAOL x = %d, y = %d", mx_goal, my_goal);

    // clear the starting cell within the costmap because we know it can't be an obstacle
    // 将初始位置在地图上设置为无障碍物状态
    tf::Stamped<tf::Pose> startTfPose;
    tf::poseStampedMsgToTF(start, startTfPose);
    clearRobotCell(startTfPose, mx_start, my_start);
    mapOriginX = costmap_->getOriginX();
    mapOriginY = costmap_->getOriginY();

    // ------------------------------------------------------
    // 初始化rrt tree
    // rrt中使用rrtTree
    // rrt connect 中使用 rrtTree和rrtConTree
    // ------------------------------------------------------
    rrtTree.clear();
    rrtStartNode = new Node;
    rrtStartNode->parent = nullptr;
    rrtStartNode->children.clear();
    rrtStartNode->cell.x = mx_start; 
    rrtStartNode->cell.y = my_start;
    rrtTree.push_back(rrtStartNode);
    
    rrtConTree.clear();
    rrtConStartNode = new Node;
    rrtConStartNode->parent = nullptr;
    rrtConStartNode->children.clear();
    rrtConStartNode->cell.x = mx_goal;
    rrtConStartNode->cell.y = my_goal;
    rrtConTree.push_back(rrtConStartNode);

    rrtStarTree.clear();
    rrtStarStartNode = new Node;
    rrtStarStartNode->parent = nullptr;
    rrtStarStartNode->children.clear();
    rrtStarStartNode->cell.x = mx_start;
    rrtStarStartNode->cell.y = my_start;
    rrtStarTree.push_back(rrtStarStartNode);

    ROS_INFO("RRT tree initialized. ");

    // ------------------------------------------------------
    // 规划
    // ------------------------------------------------------
    std::vector< geometry_msgs::PoseStamped > tempPlan;
    tempPlan.clear();
    plan.clear();
    geometry_msgs::PoseStamped pose;
    Node* lastN;

    if(buildTree(whichAlgorithm))
    {
        ROS_INFO("RRT tree built. Now begin to plan");
        // -----------------------------------  
        // RRT
        // -----------------------------------  
        if(whichAlgorithm == 0)
        {
            ROS_INFO("Use algorithm of RRT");
            // 从目标节点回溯到开始节点
            tempPlan.clear();
            lastN = rrtGoalNode;
            while(lastN != nullptr)
            {
                pose.header.frame_id = global_frame_id_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = lastN->cell.x * mapResolution + mapOriginX;
                pose.pose.position.y = lastN->cell.y * mapResolution + mapOriginY;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1.0;
                
                tempPlan.push_back(pose);
                lastN = lastN->parent;
            }

            // 倒序
            plan.clear();
            for(int i=tempPlan.size()-1; i>=0; i--)
            {
                plan.push_back(tempPlan[i]);
            }
        }
        // -----------------------------------  
        // RRT Connect
        // -----------------------------------  
        else if(whichAlgorithm == 1)    
        {
            ROS_INFO("Use algorithm of RRT connect");
            lastN = rrtGoalNode;
            tempPlan.clear();
            while(lastN != nullptr)
            {
                pose.header.frame_id = global_frame_id_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = lastN->cell.x * mapResolution + mapOriginX;
                pose.pose.position.y = lastN->cell.y * mapResolution + mapOriginY;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1.0;
                
                tempPlan.push_back(pose);
                lastN = lastN->parent;
            }

            // 倒序推入到plan中
            plan.clear();
            for(int i=(int)tempPlan.size()-1; i>=0; i--)
            {
                plan.push_back(tempPlan[i]);
            }

            lastN = rrtConGoalNode;
            while(lastN != nullptr)
            {
                pose.header.frame_id = global_frame_id_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = lastN->cell.x * mapResolution + mapOriginX;
                pose.pose.position.y = lastN->cell.y * mapResolution + mapOriginY;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1.0;

                // 由于是正序，所以直接推入到plan中
                plan.push_back(pose);
                lastN = lastN->parent;
            }
        }
        // -----------------------------------  
        // RRT star
        // -----------------------------------  
        else if(whichAlgorithm == 2)    
        {
            ROS_INFO("Use algorithm of RRT Star");
            tempPlan.clear();
            lastN = rrtStarGoalNode;
            while(lastN != nullptr)
            {
                pose.header.frame_id = global_frame_id_;
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = lastN->cell.x * mapResolution + mapOriginX;
                pose.pose.position.y = lastN->cell.y * mapResolution + mapOriginY;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1.0;
                
                tempPlan.push_back(pose);
                lastN = lastN->parent;
            }
            // 倒序
            plan.clear();
            for(int i=tempPlan.size()-1; i>=0; i--)
            {
                plan.push_back(tempPlan[i]);
            }
        }
        else
        {
            ROS_ERROR("You should choose algorithm from 0, 1 and 2");
            return false;
        }

        // -----------------------------------  
        // 平滑
        // -----------------------------------  
        if(smoothOrNot)
        {
            ROS_INFO("Size of the unsmoothed plan = %d", int(plan.size()));
            vector<RealPoint> pathSmoothed = smoothPlan(plan);
            geometry_msgs::PoseStamped nextNode;
            int sizePathSmoothed = int(pathSmoothed.size());
            ROS_INFO("Size of the Smoothed plan = %d", sizePathSmoothed);
            plan.clear();
            for(int j=0; j<sizePathSmoothed; j++){
                nextNode.header.stamp = ros::Time::now();
                nextNode.header.frame_id = global_frame_id_;

                nextNode.pose.position.x = pathSmoothed[j].x;
                nextNode.pose.position.y = pathSmoothed[j].y;

                nextNode.pose.orientation = 
                    tf::createQuaternionMsgFromRollPitchYaw(0, 0, pathSmoothed[j].theta);
                plan.push_back(nextNode);
            }
        }
        plan.back().pose.orientation = goal.pose.orientation;
        // auto tempP = plan.back();
        // tempP.pose.orientation = goal.pose.orientation;

        return true;
    }
    else
    {
        ROS_ERROR("Cannot find the goal");
        return false;
    }
}

void MyPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
        gui_path.poses[i] = path[i];
    }

    planPub.publish(gui_path);
}

bool MyPlannerROS::buildTree(int choice)
{
    for(int i=0; i<maxIteration; i++)
    {
        if(sowTargets())
        {
            // ROS_INFO("ITER = %d, num of targets = %d", i, int(targetPoints.size()));
            for(int j=0; j<targetPoints.size(); j++)
            {
                switch (choice)
                {
                case 0:
                    if( extendRRTTree(targetPoints[j]) )
                        return true;
                    break;
                case 1:
                    if( extendRRTConectTree(targetPoints[j]) )
                        return true;
                    break;
                case 2:
                    if( extendRRTStarTree(targetPoints[j]) )
                        return true;
                    break;
                default:
                    ROS_ERROR("You should choose algorithm with num 0, 1, or 2");
                    break;
                }
            }
        }
        // ROS_INFO("Building rrt tree, iter = %d", i);
    }

    return false;
}

bool MyPlannerROS::sowTargets()
{
    if(freeSpaceCnt == 0)
    {
        ROS_WARN("There is no freespace in the map");
        return false;
    }
    else
    {
        std::random_device rd;
        double tempProb = rd() % 100 * 0.01;
        if(tempProb < randProbility)
        {
            // 随机方向
            treeGrowRandom();
        }
        else
        {
            // 目标点方向
            treeGrowGoal();
        }
        return true;
    }
}

void MyPlannerROS::treeGrowGoal()
{
    targetPoints.clear();

    std::random_device rd;
    MapCell p;
    double tempwx, tempwy;
    double tempmx, tempmy;
    
    for(int i=0; i<targetNum; i++)
    {
        // 生成一个在目标点附近的点
        // tempwx = wx_goal + (rd()%200-100)*0.01 * 0.01;
        // tempwy = wy_goal + (rd()%200-100)*0.01 * 0.01;
        tempmx = mx_goal + int(rd()%200-100) * 0.05;
        tempmy = my_goal + int(rd()%200-100) * 0.05;
        
        if(tempmx < mapSizeInCellsX && tempmy < mapSizeInCellsY && tempmx>=0 && tempmy>=0)
        {
            p.x = static_cast<unsigned int>(tempmx);
            p.y = static_cast<unsigned int>(tempmy);
            targetPoints.push_back(p);
        }
        // // 如果新生成的点在地图内
        // if(costmap_->worldToMap(tempwx, tempwy, p.x, p.y))
        // {
        //     ROS_INFO("Gaol x=%d, y=%d TARGET x=%d, y=%d", mx_goal, my_goal, p.x, p.y);
        //     targetPoints.push_back(p);
        // }
    }
    // ROS_INFO("Goal oriented target %d", int(targetPoints.size()));
}

void MyPlannerROS::treeGrowRandom()
{
    targetPoints.clear();

    std::random_device rd;
    MapCell p;
    
    for(int i=0; i<targetNum; i++)
    {
        p.x = rd() % mapSizeInCellsX;
        p.y = rd() % mapSizeInCellsY;

        targetPoints.push_back(p);        
    }
}



bool MyPlannerROS::extendRRTTree(MapCell tp)
{
    // ----------------------------------------------------
    // 在已有的树的节点中，找到距离tp最近的点
    // ----------------------------------------------------
    int minIdx;
    // auto inf = std::numeric_limits<double>::infinity();
    double minDis = DBL_MAX;
    double tempDis;
    for(int i=0; i<rrtTree.size(); i++)
    {
        // hypot 返回二范数
        tempDis = hypot(double(rrtTree[i]->cell.x)-double(tp.x),
                        double(rrtTree[i]->cell.y)-double(tp.y));
        if(tempDis < minDis)
        {
            minDis = tempDis;
            minIdx = i;
        }
    }

    // ----------------------------------------------------
    // 添加节点
    // ----------------------------------------------------
    Node * node = new Node;
    if(extendSingleNode(rrtTree[minIdx], node, tp))
    {
        // ROS_INFO("New node x=%d, y=%d, tp x=%d, y=%d", node->cell.x, node->cell.y, tp.x, tp.y);
        node->parent = rrtTree[minIdx];
        rrtTree[minIdx]->children.push_back(node);
        rrtTree.push_back(node);
        if(reachGoal(node))
        {
            rrtGoalNode = node;
            ROS_INFO("Reach the goal. ");
            return true;
        }
        else
            return false;
    }
    else
    {
        ROS_INFO("Fail to extend new node here. ");
        delete node;
        return false;
    }
}

/**
 * @brief RRT connect 树生长
 *      需要用到两个树，和rrt共用rrtTree，另一个树是rrtConTree
 * @param tp 
 * @return true 
 * @return false 
 */
bool MyPlannerROS::extendRRTConectTree(MapCell tp)
{
    // ------------------------------------------
    // 第一棵树中找到离方向点tp最近的节点
    // ------------------------------------------
    int minIdx1;
    double minDis1 = DBL_MAX;
    double tempDis1;
    for(int i=0; i<(int)rrtTree.size(); i++)
    {
        tempDis1 = hypot(double(rrtTree[i]->cell.x)-double(tp.x),
                        double(rrtTree[i]->cell.y)-double(tp.y));
        if(tempDis1 < minDis1)
        {
            minDis1 = tempDis1;
            minIdx1 = i;
        }
    }

    // ------------------------------------------
    // 第一棵树添加节点
    // ------------------------------------------
    Node * newNode1 = new Node;
    if(extendSingleNode(rrtTree[minIdx1], newNode1, tp))
    {
        newNode1->parent = rrtTree[minIdx1];
        rrtTree[minIdx1]->children.push_back(newNode1);
        rrtTree.push_back(newNode1);

        // ------------------------------------------
        // 第二棵树中找到离第一棵树新节点最近的节点
        // ------------------------------------------
        int minIdx2;
        double minDis2 = DBL_MAX;
        double tempDis2;
        double tempxlen, tempylen;
        for(int i=0; i<(int)rrtConTree.size(); i++)
        {
            tempxlen = double(rrtConTree[i]->cell.x) - double(newNode1->cell.x);
            tempylen = double(rrtConTree[i]->cell.y) - double(newNode1->cell.y);
            tempDis2 = hypot(tempxlen, tempylen);
            if(tempDis2 < minDis2)
            {
                minDis2 = tempDis2;
                minIdx2 = i;
            }
        }

        Node * newNode2 = new Node;
        newNode2->cell.x = rrtConTree[minIdx2]->cell.x;
        newNode2->cell.y = rrtConTree[minIdx2]->cell.y;
        MapCell tp2;        // 第二棵树的生长方向
        tp2.x = newNode1->cell.x;
        tp2.y = newNode1->cell.y;
        if(extendSingleNode(rrtConTree[minIdx2], newNode2, tp2))
        {
            newNode2->parent = rrtConTree[minIdx2];
            rrtConTree[minIdx2]->children.push_back(newNode2);
            rrtConTree.push_back(newNode2);
            Node * newNode22;
            do{
                newNode22 = new Node;
                if( extendSingleNode(newNode2, newNode22, tp2) )
                {
                    newNode22->parent = newNode2;
                    newNode2->children.push_back(newNode22);
                    rrtConTree.push_back(newNode22);

                    newNode2 = newNode22;
                }
                else{
                    delete newNode22;
                    break;
                }
            }while( !nodesConnected(newNode1, newNode2) );
        }
        else{
            // ROS_INFO("Fail to extend new nodes in 2nd tree here. ");
            // newNode2->cell.x = rrtConTree[minIdx2]->cell.x;
            // newNode2->cell.y = rrtConTree[minIdx2]->cell.y;
            delete newNode2;
            return false;
        }

        if( nodesConnected(newNode1, newNode2) )
        {
            rrtGoalNode = newNode1;
            rrtConGoalNode = newNode2;
            return true;
        }
        else{
            // delete newNode2;
            return false;
        }

    }
    else{
        // ROS_INFO("Fail to extend new nodes in 1st tree here. ");
        delete newNode1;
        return false;
    }

}

bool MyPlannerROS::extendRRTStarTree(MapCell tp)
{
    // ----------------------------------------------------
    // 在已有的树的节点中，找到距离tp最近的点
    // ----------------------------------------------------
    // ROS_INFO("Find the nearest point to the target");
    int minIdx;
    double minDis = DBL_MAX;
    double tempDis;
    for(int i=0; i<rrtStarTree.size(); i++)
    {
        // hypot 返回二范数
        tempDis = hypot(double(rrtStarTree[i]->cell.x)-double(tp.x),
                        double(rrtStarTree[i]->cell.y)-double(tp.y));
        if(tempDis < minDis)
        {
            minDis = tempDis;
            minIdx = i;
        }
    }

    // ----------------------------------------------------
    // 添加节点
    // ----------------------------------------------------
    Node * node = new Node;
    std::vector<int> nearIdx;
    nearIdx.clear();
    double tempxlen, tempylen;
    if(extendSingleNode(rrtStarTree[minIdx], node, tp))
    {
        // -----------------------------------------
        // 寻找新节点附近一定范围内能使代价最小的父节点
        // -----------------------------------------
        // ROS_INFO("Find the nodes lying inside the range of the new node");
        for(int i=0; i<(int)rrtStarTree.size(); i++)
        {
            tempxlen = double(rrtStarTree[i]->cell.x) - double(node->cell.x);
            tempylen = double(rrtStarTree[i]->cell.y) - double(node->cell.y);
            if( (hypot(tempxlen, tempylen) < rrtStarRange) )
            {
                nearIdx.push_back(i);
            }
        }
        if(nearIdx.size()==0){
            ROS_ERROR("The rrt star parameter is not suitable");
            return false;
        }

        // ROS_INFO("Find the parent node that makes cost least");

        Node * minNode = rrtStarTree[minIdx]; // nearest
        double minCost = calculateCost(rrtStarTree[minIdx]) 
                    + calculateLineCost(rrtStarTree[minIdx], node);
        double tempCost;
        int minCostIdx = 0;
        for(auto i:nearIdx)
        {
            tempCost = calculateCost(rrtStarTree[i]) 
                    + calculateLineCost(rrtStarTree[i], node);
            if( (tempCost < minCost) && ( collisionFree(rrtStarTree[i], node) ) )
            {
                minCost = tempCost;
                minNode = rrtStarTree[i];
                minCostIdx++;
            }
        }

        node->parent = minNode;
        minNode->children.push_back(minNode);
        rrtStarTree.push_back(node);
        nearIdx.erase(nearIdx.begin() + minCostIdx);

        // -----------------------------------------
        // 重新布线
        // -----------------------------------------
        for(auto i:nearIdx)
        {
            tempCost = calculateCost(node) + 
                    calculateLineCost(node, rrtStarTree[i]);
            if(tempCost < calculateCost(rrtStarTree[i]) 
                && collisionFree(node, rrtStarTree[i]) )
            {
                // 这里还应该把rrtStarTree[i]->parent->children中的rrtStarTree[i]去掉
                auto iter = rrtStarTree[i]->parent->children.begin();
                while(iter != rrtStarTree[i]->parent->children.end())
                {
                    if( (*iter)->cell.x == rrtStarTree[i]->cell.x && 
                        (*iter)->cell.y == rrtStarTree[i]->cell.y )
                        iter = rrtStarTree[i]->parent->children.erase(iter);
                    else
                        iter ++;
                }
                rrtStarTree[i]->parent = node;
                node->children.push_back(rrtStarTree[i]);
            }
        }

        if(reachGoal(node))
        {
            rrtStarGoalNode = node;
            ROS_INFO("Reach the goal. ");
            return true;
        }
        else
            return false;
    }
    else
    {
        // ROS_INFO("Fail to extend new node here. ");
        delete node;
        return false;
    }

}


double MyPlannerROS::calculateLineCost(Node * nd1, Node * nd2)
{
    double tempxlen, tempylen;
    tempxlen = double(nd1->cell.x) - double(nd2->cell.x);
    tempylen = double(nd1->cell.y) - double(nd2->cell.y);

    double cost = hypot( tempxlen, tempylen );
    return cost;
}

double MyPlannerROS::calculateCost(Node * nd)
{
    double cost = 0;
    Node * lastN = nd;
    while(lastN->parent != nullptr)
    {
        cost += calculateLineCost(lastN->parent, lastN);
        lastN = lastN->parent;
    }
    return cost;
}

bool MyPlannerROS::collisionFree(Node * nd1, Node * nd2)
{
    double tempxlen, tempylen;
    tempxlen = double(nd1->cell.x) - double(nd2->cell.x);
    tempylen = double(nd1->cell.y) - double(nd2->cell.y);
    double th = atan2(tempylen, tempxlen);
    
    double dis = hypot( tempxlen, tempylen );
    double d = 0;

    double tempx = double(nd1->cell.x);
    double tempy = double(nd1->cell.y);

    double tempCost = 0;

    while(d < dis)
    {
        tempCost = costmap_->getCost(static_cast<unsigned int>(tempx),
                                    static_cast<unsigned int>(tempy) );
        if(tempCost >= obstacleThreshold)
        {
            ROS_DEBUG("There's obstacle on the way");
            return false;
        }
        d = d + checkStepLen/mapResolution;
        tempx = nd1->cell.x + d*cos(th);
        tempy = nd1->cell.y + d*sin(th);
    }
    return true;
}


bool MyPlannerROS::extendSingleNode(
    Node* nearestNode, Node* newNode, const MapCell & target)
{
    double tempxl = double(target.x) - double(nearestNode->cell.x);
    double tempyl = double(target.y) - double(nearestNode->cell.y);
    double th = atan2(tempyl, tempxl);      // 方向

    // 新节点的位置（未验证）
    double newx = double(nearestNode->cell.x) + stepLen/mapResolution*cos(th);
    double newy = double(nearestNode->cell.y) + stepLen/mapResolution*sin(th);

    // 用来遍历的
    double tempx = double(nearestNode->cell.x);
    double tempy = double(nearestNode->cell.y);

    // --------------------------------------------------------
    // 检查新的节点以及延伸出去的路径是否符合要求
    // --------------------------------------------------------
    if(newx<0 || newy<0)
        return false;
    
    uint8_t tempCost;

    double dis = hypot(newx-tempx, newy-tempy);     // 计算新路径的长度(在map上)
    double d = 0;

    // ROS_INFO("NEW x=%lf, y=%lf, dis=%lf", newx, newy, dis);


    // 检查新生成的路径上是否有障碍物
    while(d<dis)
    {
        tempCost = costmap_->getCost(static_cast<unsigned int>(tempx),
                                    static_cast<unsigned int>(tempy) );
        // ROS_INFO("Temp cost = %d", tempCost);
        if(tempCost >= obstacleThreshold)
        {
            ROS_DEBUG("There's obstacle on the way");
            return false;
        }
        d = d + checkStepLen/mapResolution;
        tempx = nearestNode->cell.x + d*cos(th);
        tempy = nearestNode->cell.y + d*sin(th);
    }

    tempCost = costmap_->getCost(static_cast<unsigned int>(newx),
                                        static_cast<unsigned int>(newy) );
    if(tempCost<obstacleThreshold)
    {
        newNode->cell.x = static_cast<unsigned int>(newx);
        newNode->cell.y = static_cast<unsigned int>(newy);
        return true;
    }
    else
    {
        ROS_DEBUG("The new point is in the obstacle");
        return false;
    }
}

bool MyPlannerROS::reachGoal(const Node * nd)
{
    double tempxl = double(nd->cell.x) - double(mx_goal);
    double tempyl = double(nd->cell.y) - double(my_goal);
    // ROS_INFO("dis to the goal = %lf", hypot(tempxl, tempyl));
    if(hypot(tempxl, tempyl)*mapResolution < withinDis)
        return true;
    else
        return false;
}

bool MyPlannerROS::nodesConnected(const Node * nd1, const Node * nd2)
{
    double tempxl = double(nd1->cell.x) - double(nd2->cell.x);
    double tempyl = double(nd1->cell.y) - double(nd2->cell.y);
    // ROS_INFO("dis to the goal = %lf", hypot(tempxl, tempyl));
    if(hypot(tempxl, tempyl)*mapResolution < withinDis)
        return true;
    else
        return false;
}


std::vector<RealPoint> MyPlannerROS::smoothPlan(
    std::vector<geometry_msgs::PoseStamped>& path)
{
    geometry_msgs::PoseStamped pose;
    
    int pathSize = (int)path.size();
    vector<RealPoint> inputPath;
    inputPath.clear();
    if(pathSize == 0)
    {
        ROS_ERROR("Path not valid for smoothing");
        return inputPath;
    }

    double t, oldX, oldY, oldTh, dt;
    dt = 0.1;
    int cnt = 0;
    double x, y;
    RealPoint p, pNew;
    for(auto i:path)
    {
        x = i.pose.position.x;
        y = i.pose.position.y;

        if(cnt > 0)
        {
            t = dt;
            while(t < 1){
                pNew.x = (x - oldX)*t + oldX;
                pNew.y = (y - oldY)*t + oldY;
                pNew.theta = 0;
                inputPath.push_back(pNew);
                t += dt;
            }
        }
        else
        {
            p.x = x;
            p.y = y;
            p.theta = 0;
            inputPath.push_back(p);
        }
        oldX = x;
        oldY = y;
        oldTh = 0;
        cnt ++;
    }
    
    if(pathSize < 3){
        ROS_INFO("Returning path, without smoothing it");
        return inputPath;
    }

    // providing the path to the smoother
    splineSmoother->readPathFromStruct(inputPath);
    // Filtering the path
    splineSmoother->filterPath(1);
    // Smoothing the path
    splineSmoother->smoothWhileDistanceLessThan(0.05, 1.01);
    vector<RealPoint> smoothPath = splineSmoother->getSmoothPath();

    return smoothPath;
}


} // end namespace

