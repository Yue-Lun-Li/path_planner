#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include <hybrid_astar/occupancy.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();


  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template<typename T> bool isTraversable(const T* node) const {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // 2D collision test
    if (t == 99) {
      return !grid->data[node->getIdx()];
    }

    if (true) {
      cost = configurationTest(x, y, t) ? 0 : 1;
    } else {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }
  nav_msgs::OccupancyGrid::Ptr getGrid() const {return grid;}
  hybrid_astar::occupancy::Ptr convertToExtendedGrid() {
        if (!grid) {
            return nullptr; // 如果原始grid为空，返回nullptr
        }
        auto extendedGrid = boost::make_shared<hybrid_astar::occupancy>();
        extendedGrid->header = grid->header;
        extendedGrid->info = grid->info;
        extendedGrid->data.resize(grid->data.size());

        // 这里假设你想要直接转换数据，或进行一些修改
        for (size_t i = 0; i < grid->data.size(); ++i) {
            // 根据你的需要调整数据，比如这里将数据转换为 uint8_t 范围
            extendedGrid->data[i] = static_cast<uint8_t>(grid->data[i] + 128);
        }
        return extendedGrid;
    }

  // 更新扩展栅格地图的方法
  void updateExtendedGrid(const hybrid_astar::occupancy::ConstPtr& extendedGridPtr) {
      extendedGrid = extendedGridPtr;
  }
  hybrid_astar::occupancy::ConstPtr getExtendedGrid() const {
      return extendedGrid;
  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) const {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t) const;

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}
 

  
 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  hybrid_astar::occupancy::ConstPtr extendedGrid;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
