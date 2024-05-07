#include "collisiondetection.h"
#include "algorithm.h"
using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
  const int HEIGHT_THRESHOLD = 255;  // 假设99是不可通行的高度阈值
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      // if (grid->data[cY * grid->info.width + cX]) {
      //   return false;
      // }
            // 检查网格单元的高度
      int height = grid->data[cY * grid->info.width + cX];
      if (height < 0) {
        height = height +256;
      }
      if (height > HEIGHT_THRESHOLD) {
        return false;  // 如果超过阈值，判断为不可通行
      }
    }
  }

  return true;
}