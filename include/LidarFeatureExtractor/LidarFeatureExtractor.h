#ifndef LIO_LIVOX_LIDARFEATUREEXTRACTOR_H
#define LIO_LIVOX_LIDARFEATUREEXTRACTOR_H

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <future>
#include "opencv2/core.hpp"
#include "segment/segment.hpp"

/**
 * @brief LidarFeatureExtractor类用于从激光雷达点云数据中提取特征点，包括角点和平面点。
 */
class LidarFeatureExtractor
{
  typedef pcl::PointXYZINormal PointType; // 定义点的类型，包括XYZ、反射率和法线信息。

public:
  /**
   * @brief LidarFeatureExtractor的构造函数
   * @param[in] n_scans 激光雷达的线数，用于特征提取
   * @param[in] NumCurvSize 曲率窗口大小阈值
   * @param[in] DistanceFaraway 特征点的距离阈值
   * @param[in] NumFlat 平面点的数量限制
   * @param[in] PartNum 点云分块数量
   * @param[in] FlatThreshold 平面判断的阈值
   * @param[in] BreakCornerDis 角点断点距离阈值
   * @param[in] LidarNearestDis 最近距离阈值
   * @param[in] KdTreeCornerOutlierDis KD树角点离群点距离阈值
   */
  LidarFeatureExtractor(int n_scans, int NumCurvSize, float DistanceFaraway, int NumFlat, int PartNum, float FlatThreshold,
                        float BreakCornerDis, float LidarNearestDis, float KdTreeCornerOutlierDis);

  /**
   * @brief 将浮点数转换为整数
   * @param[in] f 要转换的浮点数
   * @return 转换后的整数值
   */
  static uint32_t _float_as_int(float f);

  /**
   * @brief 将整数转换为浮点数
   * @param[in] i 要转换的整数
   * @return 转换后的浮点数值
   */
  static float _int_as_float(uint32_t i);

  /**
   * @brief 判断一组点是否属于平面
   * @param[in] point_list 点的集合
   * @param[in] plane_threshold 平面判断的阈值
   * @return 是否为平面（true为平面，false为非平面）
   */
  bool plane_judge(const std::vector<PointType> &point_list, const int plane_threshold);

  /**
   * @brief 检测激光雷达点云的特征点
   * @param[in] cloud 输入的点云
   * @param[out] pointsLessSharp 输出较少的角点索引
   * @param[out] pointsLessFlat 输出较少的平面点索引
   */
  void detectFeaturePoint(pcl::PointCloud<PointType>::Ptr &cloud,
                          std::vector<int> &pointsLessSharp,
                          std::vector<int> &pointsLessFlat);

  /**
   * @brief 检测激光雷达点云的特征点（另一种实现方式）
   * @param[in] cloud 输入的点云
   * @param[out] pointsLessFlat 输出较少的平面点
   * @param[out] pointsNonFeature 输出非特征点
   */
  void detectFeaturePoint2(pcl::PointCloud<PointType>::Ptr &cloud,
                           pcl::PointCloud<PointType>::Ptr &pointsLessFlat,
                           pcl::PointCloud<PointType>::Ptr &pointsNonFeature);

  /**
   * @brief 检测激光雷达点云的角点
   * @param[in] cloud 输入的点云
   * @param[out] pointsLessSharp 输出较少的角点索引
   */
  void detectFeaturePoint3(pcl::PointCloud<PointType>::Ptr &cloud,
                           std::vector<int> &pointsLessSharp);

  /**
   * @brief 通过分割提取激光雷达特征点
   * @param[in] msg 输入的Livox自定义消息
   * @param[out] laserCloud 输出的点云
   * @param[out] laserConerFeature 输出的角点特征
   * @param[out] laserSurfFeature 输出的平面点特征
   * @param[out] laserNonFeature 输出的非特征点
   * @param[out] msg2 输出的分割后的消息
   * @param[in] Used_Line 使用的激光雷达线数（默认为1）
   */
  void FeatureExtract_with_segment(const livox_ros_driver::CustomMsgConstPtr &msg,
                                   pcl::PointCloud<PointType>::Ptr &laserCloud,
                                   pcl::PointCloud<PointType>::Ptr &laserConerFeature,
                                   pcl::PointCloud<PointType>::Ptr &laserSurfFeature,
                                   pcl::PointCloud<PointType>::Ptr &laserNonFeature,
                                   sensor_msgs::PointCloud2 &msg2,
                                   int Used_Line = 1);

  /**
   * @brief 通过分割提取激光雷达特征点（优化实现）
   * @param[in] msg 输入的Livox自定义消息
   * @param[out] laserCloud 输出的点云
   * @param[out] laserConerFeature 输出的角点特征
   * @param[out] laserSurfFeature 输出的平面点特征
   * @param[out] laserNonFeature 输出的非特征点
   * @param[out] msg2 输出的分割后的消息
   * @param[in] Used_Line 使用的激光雷达线数（默认为1）
   */
  void FeatureExtract_with_segment_hap(const livox_ros_driver::CustomMsgConstPtr &msg,
                                       pcl::PointCloud<PointType>::Ptr &laserCloud,
                                       pcl::PointCloud<PointType>::Ptr &laserConerFeature,
                                       pcl::PointCloud<PointType>::Ptr &laserSurfFeature,
                                       pcl::PointCloud<PointType>::Ptr &laserNonFeature,
                                       sensor_msgs::PointCloud2 &msg2,
                                       int Used_Line = 1);

  /**
   * @brief 检测Livox自定义消息的特征点
   * @param[in] msg 输入的Livox自定义消息
   * @param[out] laserCloud 转换后的点云
   * @param[out] laserConerFeature 提取的角点特征
   * @param[out] laserSurfFeature 提取的平面点特征
   */
  void FeatureExtract(const livox_ros_driver::CustomMsgConstPtr &msg,
                      pcl::PointCloud<PointType>::Ptr &laserCloud,
                      pcl::PointCloud<PointType>::Ptr &laserConerFeature,
                      pcl::PointCloud<PointType>::Ptr &laserSurfFeature,
                      int Used_Line = 1, const int lidar_type = 0);

  // 其他相关函数省略...

private:
  const int N_SCANS;                                   // 激光雷达的线数
  std::vector<pcl::PointCloud<PointType>::Ptr> vlines; // 每条激光线的点云数据
  std::vector<std::vector<int>> vcorner;               // 每条激光线的角点索引
  std::vector<std::vector<int>> vsurf;                 // 每条激光线的平面点索引

  int thNumCurvSize;       // 曲率窗口大小阈值
  float thDistanceFaraway; // 特征点的距离阈值
  int thNumFlat;           // 平面点数量限制
  int thPartNum;           // 点云分块数量
  float thFlatThreshold;   // 平面判断阈值
  float thBreakCornerDis;  // 角点断点距离阈值
  float thLidarNearestDis; // 最近距离阈值
};

#endif // LIO_LIVOX_LIDARFEATUREEXTRACTOR_H
