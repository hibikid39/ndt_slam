#ifndef PCFILTER_H_
#define PCFILTER_H_

#include <string>
#include <vector>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>

#include "MyUtil.h"
#include "PCLUtil.h"

class PCFilter {
public:
  double resol;  // 差分抽出の解像度
  double thre_neighbor;
  
  PCFilter() : resol(0.05), thre_neighbor(0.1) {
    ros::param::get("resol", resol);
    ros::param::get("thre_neighbor", thre_neighbor);
  }

  ~PCFilter() {
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_neighborPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr point_list) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    removed_cloud->width = cloud_base->points.size();
    removed_cloud->height = 1;
    removed_cloud->points.resize(removed_cloud->width * removed_cloud->height);   

    int cnt = 0;
    for (size_t i = 0; i < cloud_base->points.size(); i++) {
      bool flag = true;
      for (size_t j = 0; j < point_list->points.size(); j++) {
        if (PCLUtil::distance_points(cloud_base->points[i], point_list->points[j]) < thre_neighbor) flag = false;
      }
      if (flag == true) {
        removed_cloud->points[cnt].x = cloud_base->points[i].x;
        removed_cloud->points[cnt].y = cloud_base->points[i].y;
        removed_cloud->points[cnt].z = cloud_base->points[i].z;
        cnt++;
      }
    }
    //差分点群のサイズの再設定
    removed_cloud->width = cnt;
    removed_cloud->height = 1;
    removed_cloud->points.resize(removed_cloud->width * removed_cloud->height);

    return removed_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr difference_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_test) {
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resol);//Octreeを作成

    octree.setInputCloud(cloud_base);//元となる点群を入力
    octree.addPointsFromInputCloud ();

    octree.switchBuffers();//バッファの切り替え

    octree.setInputCloud(cloud_test);//比較対象の点群を入力
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    octree.getPointIndicesFromNewVoxels(newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZ>);

    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    int n = 0;//差分点群の数を保存する
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        n++;
    }

    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    return cloud_diff;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through(std::string axis, double min, double max, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(min, max);
    pass.filter(*cloud_filtered);

    return cloud_filtered;
  }

};

#endif
