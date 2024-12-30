#include "LidarFeatureExtractor/LidarFeatureExtractor.h"

typedef pcl::PointXYZINormal PointType;

ros::Publisher pubFullLaserCloud;
ros::Publisher pubSharpCloud;
ros::Publisher pubFlatCloud;
ros::Publisher pubNonFeature;

LidarFeatureExtractor *lidarFeatureExtractor;
pcl::PointCloud<PointType>::Ptr laserCloud;
pcl::PointCloud<PointType>::Ptr laserConerCloud;
pcl::PointCloud<PointType>::Ptr laserSurfCloud;
pcl::PointCloud<PointType>::Ptr laserNonFeatureCloud;
int Lidar_Type = 0;
int N_SCANS = 6;
bool Feature_Mode = false;
bool Use_seg = false;

/**
 * @brief Livox Horizon激光雷达数据的回调函数，处理并提取点云特征。
 *
 * @param msg Livox ROS驱动程序发布的自定义消息，包含点云数据及其他信息。
 */
void lidarCallBackHorizon(const livox_ros_driver::CustomMsgConstPtr &msg)
{
  // 声明一个用于存储转换后点云数据的ROS消息
  sensor_msgs::PointCloud2 msg2;

  // 根据是否使用分段处理（Use_seg）决定特征提取方法
  if (Use_seg)
  {
    // 使用带有分段的特征提取方法
    lidarFeatureExtractor->FeatureExtract_with_segment(
        msg,                  // 输入的Livox自定义消息
        laserCloud,           // 输出的完整点云
        laserConerCloud,      // 输出的角点特征点云
        laserSurfCloud,       // 输出的平面特征点云
        laserNonFeatureCloud, // 输出的非特征点云
        msg2,                 // 转换后的点云消息
        N_SCANS               // 激光雷达的线束数
    );
  }
  else
  {
    // 使用普通的特征提取方法
    lidarFeatureExtractor->FeatureExtract(
        msg,             // 输入的Livox自定义消息
        laserCloud,      // 输出的完整点云
        laserConerCloud, // 输出的角点特征点云
        laserSurfCloud,  // 输出的平面特征点云
        N_SCANS,         // 激光雷达的线束数
        Lidar_Type       // 激光雷达的类型，用于特征提取的参数调整
    );
  }

  // 将提取特征后的完整点云转换为ROS的PointCloud2格式消息
  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*laserCloud, laserCloudMsg);

  // 设置消息头信息，包括帧ID和时间戳
  laserCloudMsg.header = msg->header;
  laserCloudMsg.header.stamp.fromNSec(msg->timebase + msg->points.back().offset_time);

  // 发布完整点云消息，供其他节点订阅使用
  pubFullLaserCloud.publish(laserCloudMsg);
}

void lidarCallBackHAP(const livox_ros_driver::CustomMsgConstPtr &msg)
{

  sensor_msgs::PointCloud2 msg2;

  if (Use_seg)
  {
    lidarFeatureExtractor->FeatureExtract_with_segment_hap(msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, msg2, N_SCANS);
  }
  else
  {
    lidarFeatureExtractor->FeatureExtract_hap(msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, N_SCANS);
  }

  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*laserCloud, laserCloudMsg);
  laserCloudMsg.header = msg->header;
  laserCloudMsg.header.stamp.fromNSec(msg->timebase + msg->points.back().offset_time);
  pubFullLaserCloud.publish(laserCloudMsg);
}

/**
 * @brief 激光雷达点云的回调函数，处理并提取特征点。
 *
 * @param msg ROS格式的PointCloud2消息，包含激光雷达的点云数据。
 */
void lidarCallBackPc2(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // 创建一个PCL点云指针，用于存储原始点云数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  // 创建一个自定义点云指针，用于存储添加了自定义属性的点云数据
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr laser_cloud_custom(new pcl::PointCloud<pcl::PointXYZINormal>());

  // 将ROS格式的点云消息转换为PCL格式
  pcl::fromROSMsg(*msg, *laser_cloud);

  // 遍历点云数据，对每个点添加自定义属性
  for (uint64_t i = 0; i < laser_cloud->points.size(); i++)
  {
    auto p = laser_cloud->points.at(i); // 获取当前点
    pcl::PointXYZINormal p_custom;      // 自定义点对象

    // 根据激光雷达类型筛选点
    if (Lidar_Type == 0 || Lidar_Type == 1)
    {
      if (p.x < 0.01) // 排除x坐标过小的点
        continue;
    }
    else if (Lidar_Type == 2)
    {
      if (std::fabs(p.x) < 0.01) // 排除x坐标绝对值过小的点
        continue;
    }

    // 复制点的基本属性（坐标和强度）
    p_custom.x = p.x;
    p_custom.y = p.y;
    p_custom.z = p.z;
    p_custom.intensity = p.intensity;

    // 添加自定义属性
    p_custom.normal_x = float(i) / float(laser_cloud->points.size()); // 正则化的点索引
    p_custom.normal_y = i % 4;                                        // 点的分组标记
    // 将自定义点添加到自定义点云中
    laser_cloud_custom->points.push_back(p_custom);
  }

  // 调用特征提取器，提取角点和面点特征
  lidarFeatureExtractor->FeatureExtract_Mid(laser_cloud_custom, laserConerCloud, laserSurfCloud);

  // 将处理后的自定义点云转换为ROS消息格式
  sensor_msgs::PointCloud2 laserCloudMsg;
  pcl::toROSMsg(*laser_cloud_custom, laserCloudMsg);

  // 保留原始消息的头信息（例如时间戳和帧ID）
  laserCloudMsg.header = msg->header;

  // 发布处理后的点云
  pubFullLaserCloud.publish(laserCloudMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ScanRegistration");
  ros::NodeHandle nodeHandler("~");

  ros::Subscriber customCloud, pc2Cloud;

  std::string config_file;
  int msg_type = 0;
  nodeHandler.getParam("config_file", config_file);
  nodeHandler.getParam("msg_type", msg_type);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    std::cout << "config_file error: cannot open " << config_file << std::endl;
    return false;
  }
  Lidar_Type = static_cast<int>(fsSettings["Lidar_Type"]);
  N_SCANS = static_cast<int>(fsSettings["Used_Line"]);
  Feature_Mode = static_cast<int>(fsSettings["Feature_Mode"]);
  Use_seg = static_cast<int>(fsSettings["Use_seg"]);

  int NumCurvSize = static_cast<int>(fsSettings["NumCurvSize"]);
  float DistanceFaraway = static_cast<float>(fsSettings["DistanceFaraway"]);
  int NumFlat = static_cast<int>(fsSettings["NumFlat"]);
  int PartNum = static_cast<int>(fsSettings["PartNum"]);
  float FlatThreshold = static_cast<float>(fsSettings["FlatThreshold"]);
  float BreakCornerDis = static_cast<float>(fsSettings["BreakCornerDis"]);
  float LidarNearestDis = static_cast<float>(fsSettings["LidarNearestDis"]);
  float KdTreeCornerOutlierDis = static_cast<float>(fsSettings["KdTreeCornerOutlierDis"]);

  laserCloud.reset(new pcl::PointCloud<PointType>);
  laserConerCloud.reset(new pcl::PointCloud<PointType>);
  laserSurfCloud.reset(new pcl::PointCloud<PointType>);
  laserNonFeatureCloud.reset(new pcl::PointCloud<PointType>);

  if (Lidar_Type == 0)
  {
    customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHorizon);
  }
  else if (Lidar_Type == 1)
  {
    customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHAP);
  }
  else if (Lidar_Type == 2)
  {
    if (msg_type == 0)
      customCloud = nodeHandler.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 100, &lidarCallBackHorizon);
    else if (msg_type == 1)
      pc2Cloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &lidarCallBackPc2);
  }
  pubFullLaserCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_full_cloud", 10);
  pubSharpCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_sharp_cloud", 10);
  pubFlatCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_less_flat_cloud", 10);
  pubNonFeature = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_nonfeature_cloud", 10);

  lidarFeatureExtractor = new LidarFeatureExtractor(N_SCANS, NumCurvSize, DistanceFaraway, NumFlat, PartNum,
                                                    FlatThreshold, BreakCornerDis, LidarNearestDis, KdTreeCornerOutlierDis);

  ros::spin();

  return 0;
}
