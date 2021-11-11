#ifndef PCLUTIL_H_
#define PCLUTIL_H_

#include <vector>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>

#include "MyUtil.h"
class PCLUtil {
public:
  PCLUtil() {
  }

  ~PCLUtil() {
  }
  
  static double distance_points(pcl::PointXYZ p1, pcl::PointXYZ p2) {
      return std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
  }
};

#endif
