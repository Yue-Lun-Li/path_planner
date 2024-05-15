#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
void updateV(Node3D& node,Node3D& nextnNode, Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace);
float updateRoll(Node3D& node, CollisionDetection& configurationSpace);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################


Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization,
                               Map* costMap ) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // // VISUALIZATION DELAY
  // ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue O;


  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);

  // update v value
  // updateV(start,configurationSpace);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  std::cout << "start seach by hybrid_astar" << std::endl;
  // continue until O empty
  while (!O.empty()) {

    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;
    // //RViz visualization
    ros::Duration d(0.003);
    // RViz visualization
    if (Constants::visualization) {
      // std::cout << "visualization.publishNode3DPoses" << std::endl;
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      // d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        std::cout << "return nPred "<< std::endl;
        return nPred;
      }
        // ____________________
        // CONTINUE WITH SEARCH
      else { 
        // _______________________
        // SEARCH WITH DUBINS SHOT
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {
            std::cout << "return dubinsShot nSucc" << std::endl;
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);
          // std::cout << "isTraversable = " <<configurationSpace.isTraversable(nSucc)<< std::endl;
          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {




              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
                //update v value
              
                updateV(*nPred,*nSucc,start,goal, configurationSpace);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  // std::cout << "(delete C) = (" << nSucc->getC() << ", " << nPred->getC() << ")" << std::endl;
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;

              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        } 
      }
    
      if(iterations > Constants::iterations){
        std::cout << "over iterations"<< std::endl;                
        return nullptr;
      }
    }
  }

  // if (O.empty()) {
  //   std::cout << "return empty nullptr" << std::endl;  
  //   return nullptr;
  // }
  std::cout << "please choose correct start and goal" << std::endl;  
  return nullptr;
}
//###################################################
//                                         GREY
//###################################################
void updateV(Node3D& node, Node3D& nextNode,Node3D& start,const Node3D& goal, CollisionDetection& configurationSpace) {  
    auto extendedGrid = configurationSpace.convertToExtendedGrid();
    if (!extendedGrid) {
        std::cerr << "Failed to get extended grid" << std::endl;
        return;
    }

    // 从这里开始，使用 extendedGrid 替代 grid
    int idx = (int)(node.getY()) * extendedGrid->info.width + (int)(node.getX());
    if (idx < 0 || idx >= extendedGrid->info.width * extendedGrid->info.height) {
        std::cerr << "Index out of range error" << std::endl;
        return;
    }
    
    // 获取当前节点和下一个节点的坐标
    double resolution = extendedGrid->info.resolution;
    float currentX = node.getX();
    float currentY = node.getY();
    float nextX = nextNode.getX();
    float nextY = nextNode.getY();
    
    // 计算当前节点和下一个节点之间的水平距离
    float horizontalDistance = std::sqrt(std::pow(nextX - currentX, 2) + std::pow(nextY - currentY, 2)) * resolution;
    
    // 计算高度差
    int currentHeight = extendedGrid->data[idx];
    int nextIdx = (int)(nextY) * extendedGrid->info.width + (int)(nextX);
    int nextHeight = extendedGrid->data[nextIdx];
    float heightDifference = (nextHeight - currentHeight)*(float)5.2209/(float)255;
    
    // 计算俯仰角和横滚角（假设以水平面为基准）
    float nextNodepitch = std::atan(heightDifference / horizontalDistance) ; // 俯仰角
    float nextNoderoll = updateRoll(nextNode, configurationSpace); // 假设横滚角为0（即以水平面为基准）
    nextNode.setP(nextNodepitch);
    nextNode.setR(nextNoderoll);

    float nodepitch = node.getP();
    float noderoll = node.getR();
    
    float dpitch = nextNodepitch - nodepitch;
    float droll = nextNoderoll - noderoll;


    float distanceToGoal = std::sqrt(std::pow(goal.getX() - nextX, 2) + std::pow(goal.getY() - nextY, 2)) ;
    float distanceForAll = std::sqrt(std::pow(goal.getX() - start.getX(), 2) + std::pow(goal.getY() - start.getY(), 2)) ;
    float normalizedDistance = distanceToGoal / distanceForAll; // 假设width是地图的宽度

  

    // 将俯仰角和横滚角作为代价值更新节点的V值（假设V值的更新方式为加和）
    float cost =4 *normalizedDistance * (std::abs(dpitch) + std::abs(droll)) *180 / M_PI; // 将俯仰角和横滚角绝对值相加作为代价

    nextNode.setV(cost);
    float currentV = nextNode.getV();
    float currentG = nextNode.getG();
    float currentH = nextNode.getH();

    std::cout << "====================" << endl;
    std::cout << "map.idx = "  << nextIdx << std::endl;
    std::cout << "node x = " << currentX <<"  y = " << currentY << std::endl;
    std::cout << "next x = " << nextX <<"  y = " << nextY << std::endl;
    std::cout << "height = " << currentHeight << std::endl;
    std::cout << "nextheight = " << nextHeight << std::endl;
    std::cout << "heightDifference = " << heightDifference << std::endl;
    std::cout << "horizontalDistance = " << horizontalDistance << std::endl;
    std::cout << "dpitch = " << dpitch << std::endl;
    std::cout << "droll = " << droll << std::endl;
    std::cout << "normalizedDistance = " << normalizedDistance << std::endl;
    std::cout << "cost = " << cost << std::endl;
    std::cout << "currentG = " << currentG << std::endl;
    std::cout << "currentH = " << currentH << std::endl;
    std::cout << "currentV = " << currentV << std::endl;

    std::cout << "====================" << endl;


}

float updateRoll(Node3D& node, CollisionDetection& configurationSpace) {
    auto grid = configurationSpace.getExtendedGrid(); // 假设已有方法获取高度信息

    float theta = node.getT(); // 获取车辆的朝向（弧度）
    float width = grid->info.width;
    float height = grid->info.height;

    // 左侧和右侧向量（假设朝向是基于正北方向，且角度增加方向为顺时针）
    float leftX = node.getX() - sin(theta);
    float leftY = node.getY() + cos(theta);
    float rightX = node.getX() + sin(theta);
    float rightY = node.getY() - cos(theta);

    int idx = (int)(node.getY()) * grid->info.width + (int)(node.getX());
    int leftIdx = (int)(leftY) *  grid->info.width + (int)(leftX);
    int rightIdx = (int)(rightY) *  grid->info.width + (int)(rightX);

    float leftHeight = grid->data[leftIdx];
    float rightHeight = grid->data[rightIdx];
    float heightDifference = (rightHeight - leftHeight)*(float)5.2209/(float)255; // 计算高度差

    float horizontalDistance = 2 * grid->info.resolution; // 假设格点间距离为resolution的两倍
    float roll = std::atan(heightDifference/ horizontalDistance) ; // 计算横滚角
    // std::cout << "====================" << endl;

    // std::cout << "idx = " << idx << std::endl;
    // std::cout << "node.x = " << node.getX() << "  node.y = " << node.getY() << std::endl;
    // std::cout << "left.x = " << leftX << "  left.y = " << leftY << std::endl;
    // std::cout << "right.x = " << rightX << "  right.y = " << rightY << std::endl;
    // std::cout << "leftIdx = " << leftIdx << std::endl;
    // std::cout << "rightIdx = " << rightIdx << std::endl;
    // std::cout << "leftHeight = " << leftHeight << std::endl;
    // std::cout << "rightHeight = " << rightHeight << std::endl;
    // std::cout << "heightDifference = " << heightDifference << std::endl;
    // std::cout << "horizontalDistance = " << horizontalDistance << std::endl;
    // std::cout << "roll = " << roll << std::endl;
    // std::cout << "====================" << endl;

    return roll;
}



// void updateV(Node3D& node,CollisionDetection& configurationSpace) {
//   nav_msgs::OccupancyGrid::Ptr grid = configurationSpace.getGrid();
//   int idx = (int)(node.getY()) * grid->info.width + (int)(node.getX());
//   int height = grid->data[idx];
//   std::cout << "====================" << endl;
//   std::cout << "map.idx = "  << idx << std::endl;
//   std::cout << "height = " << height << std::endl;
//    // node.setV( 1000 * (map[idx].getV()) );
//   // std::cout << "map.V = "  << 1000*map[idx].getV() << std::endl;
//   // std::cout << "node.v = " << node.getV() << std::endl;
//   std::cout << "====================" << endl;

// }


//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list


          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    // std::cout<<"＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝"<<endl;
    // std::cout<<"dubin cost = "<<dubinsCost<<endl;
  }

  // // if reversing is active use a
  // if (Constants::reverse && !Constants::dubins) {
  //   //    ros::Time t0 = ros::Time::now();
  //   ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
  //   State* rsStart = (State*)reedsSheppPath.allocState();
  //   State* rsEnd = (State*)reedsSheppPath.allocState();
  //   rsStart->setXY(start.getX(), start.getY());
  //   rsStart->setYaw(start.getT());
  //   rsEnd->setXY(goal.getX(), goal.getY());
  //   rsEnd->setYaw(goal.getT());
  //   reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  //   ros::Time t1 = ros::Time::now();
  //   ros::Duration d(t1 - t0);
  //   std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  // }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
       ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
       ros::Time t1 = ros::Time::now();
       ros::Duration d(t1 - t0);
      //  std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
    // std::cout<<"twoD cost = "<<twoDCost<<endl;
    // std::cout<<"＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝"<<endl;
  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}



//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  // avoid duplicate waypoint
  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      // std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
