#ifndef MY_PLANNER_CPP
#define MY_PLANNER_CPP

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <random>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>

#include <my_global_planner/pathSplineSmoother/pathSplineSmoother.h>


namespace my_global_planner{

/**
 * @brief 存储x和y位置的结构体
 * 
 */
struct MapCell{
  unsigned int x;
  unsigned int y;
};


struct Node{
  MapCell cell;
  Node * parent;
  std::vector<Node*> children;
  // // constructor 
  // Node(Pos pos_, Node *parent_ = nullptr)
  // {
  //   pos = pos_;
  //   parent = parent_;
  // }
};



  /**
   * @class MyPlannerROS
   * @brief Plugin to the ros dstar_global_planner. Implements a wrapper for DStarPlanner Method
   */
  class MyPlannerROS : public nav_core::BaseGlobalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      MyPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the dstar planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      MyPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);


      /**
       * @brief  Destructor for the wrapper
       */
      ~MyPlannerROS();

      /**
          * @brief  Initialization function for the NavFnROS object
          * @param  name The name of this planner
          * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(const std::string& name, costmap_2d::Costmap2D* costmap, std::string frame_id);


      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool getPlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


  protected:

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

      std::string global_frame_id_;

      costmap_2d::Costmap2D* costmap_;

      // pointer to the ROS wrapper of the costmap to use for planning
      costmap_2d::Costmap2DROS * costmap_ros_; 

      ros::Publisher plan_pub_;


      bool initialized_;
 

    private:

      std::string tf_prefix_;

      void mapToWorld(double mx, double my, double& wx, double& wy);

      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);

      /**
       * @brief 地图参数
       */
      double mapOriginX, mapOriginY;                // 原点
      unsigned int mapSizeInCellsX, mapSizeInCellsY;      // 地图大小
      unsigned int mapAreaInCells;          // 地图面积
      double mapResolution;           // 地图解析度，即每个像素代表多少m
      
      uint8_t obstacleThreshold;      // 把像素点判断为障碍物的cost阈值
      int freeSpaceCnt;               // 计算可行域的格子数量
      
      /**
       * @brief 起始点和终止点的位置
       */
      unsigned int mx_start, my_start;
      unsigned int mx_goal, my_goal;
      unsigned int wx_start, wy_start;
      unsigned int wx_goal, wy_goal;


      /**
       * @brief 地图初始化
       * 
       * @param costmap 
       */
      void mapInit(costmap_2d::Costmap2D* costmap);

      /**
       * @brief RRT参数
       */
      int whichAlgorithm;   // 选择哪一种算法
      int maxIteration;     // 每次建树时最大的迭代次数
      int targetNum;        // 生成目标点的数目
      double stepLen;       // 步长
      double randProbility; // 选择随机方向的概率
      double goalProbility; // 选择目标方向的概率
      double withinDis;     // 与目标点的距离小于此值，则认为到达目标点
      double checkStepLen;  // 检查新生成的分支时用到的步长
      bool smoothOrNot;     // 是否对路径规划路线进行平滑处理

      /**
       * @brief RRT变量
       */
      std::vector<Node*> rrtTree;
      Node* rrtStartNode;
      Node* rrtGoalNode;

      /**
       * @brief RRT connect 变量
       */
      std::vector<Node*> rrtConTree;
      Node* rrtConStartNode;
      Node* rrtConGoalNode;

      /**
       * @brief RRT star 参数
       */
      double rrtStarRange;

      /**
       * @brief RRT star 变量
       */
      std::vector<Node*> rrtStarTree;
      Node* rrtStarStartNode;
      Node* rrtStarGoalNode;

      // ------------------------------------------------
      // rrt相关函数以及变量
      // ------------------------------------------------
      /**
       * @brief 建立RRT tree
       * 
       * @param choice 选择哪种算法，0: rrt, 1: rrt connect 2: rrt star
       * @return true 建树成功
       * @return false 建树失败
       */
      bool buildTree(int choice);

      /**
       * @brief 播撒随机点
       * 
       * @param choice 选择哪种算法，0: rrt, 1: rrt connect 2: rrt star
       * @return true 
       * @return false 
       */
      bool sowTargets();

      /**
       * @brief 向着目标点生长
       */
      void treeGrowGoal();
      /**
       * @brief 向着随机方向生长
       */
      void treeGrowRandom();
      std::vector<MapCell> targetPoints;    // tree每一次生长时的方向

      bool extendSingleNode(Node* nearestNode, Node* newNode, const MapCell & target);

      /**
       * @brief 树生长
       * 
       * @param tp 生长方向 target point
       * @return true 
       * @return false 
       */
      bool extendRRTTree(MapCell tp);

      bool extendRRTConectTree(MapCell tp);

      bool extendRRTStarTree(MapCell tp);

      double calculateLineCost(Node * nd1, Node * nd2);
      double calculateCost(Node * nd);

      bool collisionFree(Node * nd1, Node * nd2);

      /**
       * @brief 到达目标点
       * 
       * @param nearestNode 
       * @return true 
       * @return false 
       */
      bool reachGoal(const Node * nd);

      bool nodesConnected(const Node * nd1, const Node * nd2);

      /**
       * @brief Publisher and subsriber
       */
      ros::Publisher planPub, treePub, samplePub;
      ros::Subscriber mapSub, poseSub, goalSub;

      /**
       * @brief Path Smoother
       */
      PathSplineSmoother *splineSmoother;

      std::vector<RealPoint> smoothPlan(std::vector<geometry_msgs::PoseStamped>& path);



  };
}
#endif


