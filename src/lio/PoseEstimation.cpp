#include "Estimator/Estimator.h"
typedef pcl::PointXYZINormal PointType;

int WINDOWSIZE;
bool LidarIMUInited = false;
boost::shared_ptr<std::list<Estimator::LidarFrame>> lidarFrameList;
pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
Estimator *estimator;

ros::Publisher pubLaserOdometry;
ros::Publisher pubLaserOdometryPath;
ros::Publisher pubFullLaserCloud;
tf::StampedTransform laserOdometryTrans;
tf::TransformBroadcaster *tfBroadcaster;
ros::Publisher pubGps;

bool newfullCloud = false;

Eigen::Matrix4d transformAftMapped = Eigen::Matrix4d::Identity();

std::mutex _mutexLidarQueue;
std::queue<sensor_msgs::PointCloud2ConstPtr> _lidarMsgQueue;
std::mutex _mutexIMUQueue;
std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
Eigen::Matrix4d exTlb;
Eigen::Matrix3d exRlb, exRbl;
Eigen::Vector3d exPlb, exPbl;
Eigen::Vector3d GravityVector;
float filter_parameter_corner = 0.2;
float filter_parameter_surf = 0.4;
int IMU_Mode = 2;
sensor_msgs::NavSatFix gps;
int pushCount = 0;
double startTime = 0;

nav_msgs::Path laserOdoPath;

/** \brief publish odometry infomation
 * \param[in] newPose: pose to be published
 * \param[in] timefullCloud: time stamp
 */
void pubOdometry(const Eigen::Matrix4d &newPose, double &timefullCloud)
{
  nav_msgs::Odometry laserOdometry;

  Eigen::Matrix3d Rcurr = newPose.topLeftCorner(3, 3);
  Eigen::Quaterniond newQuat(Rcurr);
  Eigen::Vector3d newPosition = newPose.topRightCorner(3, 1);
  laserOdometry.header.frame_id = "/world";
  laserOdometry.child_frame_id = "/livox_frame";
  laserOdometry.header.stamp = ros::Time().fromSec(timefullCloud);
  laserOdometry.pose.pose.orientation.x = newQuat.x();
  laserOdometry.pose.pose.orientation.y = newQuat.y();
  laserOdometry.pose.pose.orientation.z = newQuat.z();
  laserOdometry.pose.pose.orientation.w = newQuat.w();
  laserOdometry.pose.pose.position.x = newPosition.x();
  laserOdometry.pose.pose.position.y = newPosition.y();
  laserOdometry.pose.pose.position.z = newPosition.z();
  pubLaserOdometry.publish(laserOdometry);

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  laserOdoPath.header.stamp = laserOdometry.header.stamp;
  laserOdoPath.poses.push_back(laserPose);
  laserOdoPath.header.frame_id = "/world";
  pubLaserOdometryPath.publish(laserOdoPath);

  laserOdometryTrans.frame_id_ = "/world";
  laserOdometryTrans.child_frame_id_ = "/livox_frame";
  laserOdometryTrans.stamp_ = ros::Time().fromSec(timefullCloud);
  laserOdometryTrans.setRotation(tf::Quaternion(newQuat.x(), newQuat.y(), newQuat.z(), newQuat.w()));
  laserOdometryTrans.setOrigin(tf::Vector3(newPosition.x(), newPosition.y(), newPosition.z()));
  tfBroadcaster->sendTransform(laserOdometryTrans);

  gps.header.stamp = ros::Time().fromSec(timefullCloud);
  gps.header.frame_id = "world";
  gps.latitude = newPosition.x();
  gps.longitude = newPosition.y();
  gps.altitude = newPosition.z();
  gps.position_covariance = {
      Rcurr(0, 0), Rcurr(1, 0), Rcurr(2, 0),
      Rcurr(0, 1), Rcurr(1, 1), Rcurr(2, 1),
      Rcurr(0, 2), Rcurr(1, 2), Rcurr(2, 2)};
  pubGps.publish(gps);
}

void fullCallBack(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // push lidar msg to queue
  std::unique_lock<std::mutex> lock(_mutexLidarQueue);
  _lidarMsgQueue.push(msg);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  // push IMU msg to queue
  std::unique_lock<std::mutex> lock(_mutexIMUQueue);
  _imuMsgQueue.push(imu_msg);
}

/** \brief get IMU messages in a certain time interval
 * \param[in] startTime: left boundary of time interval
 * \param[in] endTime: right boundary of time interval
 * \param[in] vimuMsg: store IMU messages
 */
bool fetchImuMsgs(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimuMsg)
{
  std::unique_lock<std::mutex> lock(_mutexIMUQueue);
  double current_time = 0;
  vimuMsg.clear();
  while (true)
  {
    if (_imuMsgQueue.empty())
      break;
    if (_imuMsgQueue.back()->header.stamp.toSec() < endTime ||
        _imuMsgQueue.front()->header.stamp.toSec() >= endTime)
      break;
    sensor_msgs::ImuConstPtr &tmpimumsg = _imuMsgQueue.front();
    double time = tmpimumsg->header.stamp.toSec();
    if (time <= endTime && time > startTime)
    {
      vimuMsg.push_back(tmpimumsg);
      current_time = time;
      _imuMsgQueue.pop();
      if (time == endTime)
        break;
    }
    else
    {
      if (time <= startTime)
      {
        _imuMsgQueue.pop();
      }
      else
      {
        double dt_1 = endTime - current_time;
        double dt_2 = time - endTime;
        ROS_ASSERT(dt_1 >= 0);
        ROS_ASSERT(dt_2 >= 0);
        ROS_ASSERT(dt_1 + dt_2 > 0);
        double w1 = dt_2 / (dt_1 + dt_2);
        double w2 = dt_1 / (dt_1 + dt_2);
        sensor_msgs::ImuPtr theLastIMU(new sensor_msgs::Imu);
        theLastIMU->linear_acceleration.x = w1 * vimuMsg.back()->linear_acceleration.x + w2 * tmpimumsg->linear_acceleration.x;
        theLastIMU->linear_acceleration.y = w1 * vimuMsg.back()->linear_acceleration.y + w2 * tmpimumsg->linear_acceleration.y;
        theLastIMU->linear_acceleration.z = w1 * vimuMsg.back()->linear_acceleration.z + w2 * tmpimumsg->linear_acceleration.z;
        theLastIMU->angular_velocity.x = w1 * vimuMsg.back()->angular_velocity.x + w2 * tmpimumsg->angular_velocity.x;
        theLastIMU->angular_velocity.y = w1 * vimuMsg.back()->angular_velocity.y + w2 * tmpimumsg->angular_velocity.y;
        theLastIMU->angular_velocity.z = w1 * vimuMsg.back()->angular_velocity.z + w2 * tmpimumsg->angular_velocity.z;
        theLastIMU->header.stamp.fromSec(endTime);
        vimuMsg.emplace_back(theLastIMU);
        break;
      }
    }
  }
  return !vimuMsg.empty();
}

/** \brief Remove Lidar Distortion
 * \param[in] cloud: lidar cloud need to be undistorted
 * \param[in] dRlc: delta rotation
 * \param[in] dtlc: delta displacement
 */
void RemoveLidarDistortion(pcl::PointCloud<PointType>::Ptr &cloud,
                           const Eigen::Matrix3d &dRlc, const Eigen::Vector3d &dtlc)
{
  int PointsNum = cloud->points.size();
  for (int i = 0; i < PointsNum; i++)
  {
    Eigen::Vector3d startP;
    float s = cloud->points[i].normal_x;
    Eigen::Quaterniond qlc = Eigen::Quaterniond(dRlc).normalized();
    Eigen::Quaterniond delta_qlc = Eigen::Quaterniond::Identity().slerp(s, qlc).normalized();
    const Eigen::Vector3d delta_Plc = s * dtlc;
    startP = delta_qlc * Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z) + delta_Plc;
    Eigen::Vector3d _po = dRlc.transpose() * (startP - dtlc);

    cloud->points[i].x = _po(0);
    cloud->points[i].y = _po(1);
    cloud->points[i].z = _po(2);
    cloud->points[i].normal_x = 1.0;
  }
}

/**
 * @brief 尝试通过多传感器联合优化的方式进行初始地图与参数的初始化
 *
 * @details
 * 本函数使用IMU和LiDAR传感器的数据，结合Ceres优化框架，通过残差构建并优化初始重力方向、速度、偏置和其他相关参数。
 * 包括以下主要步骤：
 * 1. 使用IMU数据计算初始的重力方向。
 * 2. 构建先验约束，优化重力方向、速度、偏置等参数。
 * 3. 使用IMU积分的结果与位姿变化添加残差约束。
 * 4. 检查优化结果的有效性并更新所有关键帧的状态。
 *
 * @return bool 返回初始化是否成功
 * - true：初始化成功
 * - false：初始化失败
 */
bool TryMAPInitialization()
{
  // **1. 利用IMU数据估算初始重力方向**
  Eigen::Vector3d average_acc = -lidarFrameList->begin()->imuIntegrator.GetAverageAcc();
  double info_g = std::fabs(9.805 - average_acc.norm());  // 计算加速度差值
  average_acc = average_acc * 9.805 / average_acc.norm(); // 归一化重力加速度

  // 初始化重力方向参数
  double para_quat[4] = {1, 0, 0, 0};                                                // 四元数初始化为单位四元数
  ceres::LocalParameterization *quatParam = new ceres::QuaternionParameterization(); // 定义四元数的局部参数化
  ceres::Problem problem_quat;
  problem_quat.AddParameterBlock(para_quat, 4, quatParam);
  problem_quat.AddResidualBlock(Cost_Initial_G::Create(average_acc), nullptr, para_quat);

  ceres::Solver::Options options_quat;
  ceres::Solver::Summary summary_quat;
  ceres::Solve(options_quat, &problem_quat, &summary_quat);

  Eigen::Quaterniond q_wg(para_quat[0], para_quat[1], para_quat[2], para_quat[3]); // 重力方向

  // **2. 构造先验因子**
  Eigen::Vector3d prior_r = Eigen::Vector3d::Zero();
  Eigen::Vector3d prior_ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d prior_bg = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> prior_v(lidarFrameList->size(), Eigen::Vector3d::Zero());
  Sophus::SO3d SO3_R_wg(q_wg.toRotationMatrix());
  prior_r = SO3_R_wg.log(); // 使用SO3的log形式保存重力方向

  // **3. 初始化速度**
  for (size_t i = 1; i < lidarFrameList->size(); i++)
  {
    auto iter = lidarFrameList->begin();
    auto iter_next = lidarFrameList->begin();
    std::advance(iter, i - 1);
    std::advance(iter_next, i);
    prior_v[i] = (iter_next->P - iter->P + iter_next->Q * exPlb - iter->Q * exPlb) / (iter_next->timeStamp - iter->timeStamp);
  }
  prior_v[0] = prior_v[1];

  // **4. 构建优化问题**
  double para_v[lidarFrameList->size()][3];
  double para_r[3] = {0}, para_ba[3] = {0}, para_bg[3] = {0};
  for (size_t i = 0; i < lidarFrameList->size(); i++)
  {
    for (int j = 0; j < 3; j++)
    {
      para_v[i][j] = prior_v[i][j];
    }
  }

  Eigen::Matrix<double, 3, 3> sqrt_information_r = 2000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_ba = 1000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_bg = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();
  Eigen::Matrix<double, 3, 3> sqrt_information_v = 4000.0 * Eigen::Matrix<double, 3, 3>::Identity();

  ceres::Problem problem;
  problem.AddParameterBlock(para_r, 3);
  problem.AddParameterBlock(para_ba, 3);
  problem.AddParameterBlock(para_bg, 3);
  for (size_t i = 0; i < lidarFrameList->size(); i++)
  {
    problem.AddParameterBlock(para_v[i], 3);
  }

  // 添加先验残差
  problem.AddResidualBlock(Cost_Initialization_Prior_R::Create(prior_r, sqrt_information_r), nullptr, para_r);
  problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_ba, sqrt_information_ba), nullptr, para_ba);
  problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_bg, sqrt_information_bg), nullptr, para_bg);
  for (size_t i = 0; i < lidarFrameList->size(); i++)
  {
    problem.AddResidualBlock(Cost_Initialization_Prior_bv::Create(prior_v[i], sqrt_information_v), nullptr, para_v[i]);
  }

  // 添加IMU残差
  for (size_t i = 1; i < lidarFrameList->size(); i++)
  {
    auto iter = lidarFrameList->begin();
    auto iter_next = lidarFrameList->begin();
    std::advance(iter, i - 1);
    std::advance(iter_next, i);
    problem.AddResidualBlock(Cost_Initialization_IMU::Create(
                                 iter_next->imuIntegrator,
                                 Sophus::SO3d(iter->Q * exRlb).log(),
                                 Sophus::SO3d(iter_next->Q * exRlb).log(),
                                 iter_next->P + iter_next->Q * exPlb - (iter->P + iter->Q * exPlb),
                                 Eigen::LLT<Eigen::Matrix<double, 9, 9>>(iter_next->imuIntegrator.GetCovariance().block<9, 9>(0, 0).inverse()).matrixL().transpose()),
                             nullptr,
                             para_r, para_v[i - 1], para_v[i], para_ba, para_bg);
  }

  // 优化求解
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.linear_solver_type = ceres::DENSE_QR;
  options.num_threads = 6;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // **5. 检查优化结果并更新状态**
  Eigen::Vector3d r_wg(para_r[0], para_r[1], para_r[2]);
  GravityVector = Sophus::SO3d::exp(r_wg) * Eigen::Vector3d(0, 0, -9.805);
  Eigen::Vector3d ba_vec(para_ba[0], para_ba[1], para_ba[2]);
  Eigen::Vector3d bg_vec(para_bg[0], para_bg[1], para_bg[2]);

  if (ba_vec.norm() > 0.5 || bg_vec.norm() > 0.5)
  {
    ROS_WARN("Too Large Biases! Initialization Failed!");
    return false;
  }

  for (size_t i = 0; i < lidarFrameList->size(); i++)
  {
    auto iter = lidarFrameList->begin();
    std::advance(iter, i);
    iter->ba = ba_vec;
    iter->bg = bg_vec;
    Eigen::Vector3d bv_vec(para_v[i][0], para_v[i][1], para_v[i][2]);
    if ((bv_vec - prior_v[i]).norm() > 2.0)
    {
      ROS_WARN("Too Large Velocity! Initialization Failed!");
      return false;
    }
    iter->V = bv_vec;
  }

  // 更新IMU预积分
  for (size_t i = 0; i < lidarFrameList->size() - 1; i++)
  {
    auto laser_trans_i = lidarFrameList->begin();
    auto laser_trans_j = lidarFrameList->begin();
    std::advance(laser_trans_i, i);
    std::advance(laser_trans_j, i + 1);
    laser_trans_j->imuIntegrator.PreIntegration(laser_trans_i->timeStamp, laser_trans_i->bg, laser_trans_i->ba);
  }

  // 窗口管理
  WINDOWSIZE = Estimator::SLIDEWINDOWSIZE;
  while (lidarFrameList->size() > WINDOWSIZE)
  {
    lidarFrameList->pop_front();
  }
  Eigen::Vector3d Pwl = lidarFrameList->back().P;
  Eigen::Quaterniond Qwl = lidarFrameList->back().Q;
  lidarFrameList->back().P = Pwl + Qwl * exPlb;
  lidarFrameList->back().Q = Qwl * exRlb;

  return true;
}

/** \brief Mapping main thread
 */
/**
 * @brief 主处理函数，用于对 LiDAR 数据和 IMU 数据进行组合处理，并发布处理后的结果。
 *
 * @details
 * 1. 从消息队列中获取最新的 LiDAR 点云数据，转换为 PCL 格式。
 * 2. 如果已启用 IMU 模式，按照时间区间获取与该帧 LiDAR 数据对应的 IMU 数据，以进行相应的积分或预积分。
 * 3. 初始化或更新当前 LiDAR 帧的位姿、速度等信息（若已完成初始化，则使用滑窗法进行更新）。
 * 4. 去除 LiDAR 畸变，并调用外部优化器（Estimation 模块）估计当前 LiDAR 帧位姿。
 * 5. 根据优化后的 LiDAR 位姿发布对应的位姿（Odometry）和点云（sensor_msgs::PointCloud2）。
 * 6. 在启用紧耦合 IMU 的模式下，维护一个包含若干 LiDAR 帧的队列（lidarFrameList），并尝试进行联合初始化。
 *
 * @note
 * - 需要配合外部定义的变量和函数使用，如：
 *   - `_mutexLidarQueue`、`_lidarMsgQueue`：LiDAR 消息队列及其互斥锁。
 *   - `fetchImuMsgs(time_last, time_curr, vimuMsg)`：获取指定时间段内的 IMU 数据。
 *   - `lidarFrameList`：存储若干 LiDAR 帧的容器，用于滑窗或初始化。
 *   - `RemoveLidarDistortion`：去除 LiDAR 点云畸变的函数。
 *   - `Estimator`、`TryMAPInitialization`、`EstimateLidarPose`：外部的估计和初始化模块。
 *   - `transformAftMapped`：保存当前优化后的位姿变换矩阵。
 *   - `exTlb`、`exRlb`、`exPbl` 等外参变量。
 *   - `GravityVector`：全局重力向量。
 *   - `pubOdometry`、`pubFullLaserCloud`：发布里程计和点云的话题。
 */
void process()
{
  // 用于记录上一帧和当前帧 LiDAR 的时间戳
  double time_last_lidar = -1;
  double time_curr_lidar = -1;

  // LiDAR 与 Body 坐标系之间的增量位姿
  Eigen::Matrix3d delta_Rl = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_tl = Eigen::Vector3d::Zero();
  Eigen::Matrix3d delta_Rb = Eigen::Matrix3d::Identity();
  Eigen::Vector3d delta_tb = Eigen::Vector3d::Zero();

  // 缓存当前帧对应的 IMU 消息
  std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

  while (ros::ok())
  {
    newfullCloud = false; // 标记是否有新 LiDAR 数据
    laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

    // 1. 从消息队列中获取最新的 LiDAR 数据
    std::unique_lock<std::mutex> lock_lidar(_mutexLidarQueue);
    if (!_lidarMsgQueue.empty())
    {
      time_curr_lidar = _lidarMsgQueue.front()->header.stamp.toSec();
      pcl::fromROSMsg(*_lidarMsgQueue.front(), *laserCloudFullRes);
      _lidarMsgQueue.pop();
      newfullCloud = true; // 获取成功
    }
    lock_lidar.unlock();

    // 若成功获取 LiDAR 数据，则进行后续处理
    if (newfullCloud)
    {
      // 2. 生成调试信息（IMU 模式下，用于查看过程）
      nav_msgs::Odometry debugInfo;
      debugInfo.pose.pose.position.x = 0;
      debugInfo.pose.pose.position.y = 0;
      debugInfo.pose.pose.position.z = 0;

      // 如果 IMU 模式启用，并且不是第一帧，则获取与 [time_last_lidar, time_curr_lidar] 对应的 IMU 数据
      if (IMU_Mode > 0 && time_last_lidar > 0)
      {
        vimuMsg.clear();
        int countFail = 0;
        while (!fetchImuMsgs(time_last_lidar, time_curr_lidar, vimuMsg))
        {
          countFail++;
          if (countFail > 100)
          {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }

      // 3. 构造当前 LidarFrame，并填充基础信息
      Estimator::LidarFrame lidarFrame;
      lidarFrame.laserCloud = laserCloudFullRes;
      lidarFrame.timeStamp = time_curr_lidar;

      boost::shared_ptr<std::list<Estimator::LidarFrame>> lidar_list;

      // 若获取到了 IMU 消息
      if (!vimuMsg.empty())
      {
        // (1) 若尚未完成激光-IMU 初始对准，则只进行陀螺积分，获得相对旋转
        if (!LidarIMUInited)
        {
          lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
          lidarFrame.imuIntegrator.GyroIntegration(time_last_lidar);

          delta_Rb = lidarFrame.imuIntegrator.GetDeltaQ().toRotationMatrix();
          // 将 body 坐标系下的旋转变换到 LiDAR 坐标系
          delta_Rl = exTlb.topLeftCorner(3, 3) * delta_Rb * exTlb.topLeftCorner(3, 3).transpose();

          // 预测当前 LiDAR 的位置，简单累加
          lidarFrame.P = transformAftMapped.topLeftCorner(3, 3) * delta_tb + transformAftMapped.topRightCorner(3, 1);
          Eigen::Matrix3d m3d = transformAftMapped.topLeftCorner(3, 3) * delta_Rb;
          lidarFrame.Q = m3d;

          lidar_list.reset(new std::list<Estimator::LidarFrame>);
          lidar_list->push_back(lidarFrame);
        }
        else
        {
          // (2) 若已经完成激光-IMU 初始对准，则对该帧进行 IMU 预积分，并更新位姿
          lidarFrame.imuIntegrator.PushIMUMsg(vimuMsg);
          lidarFrame.imuIntegrator.PreIntegration(lidarFrameList->back().timeStamp, lidarFrameList->back().bg, lidarFrameList->back().ba);

          // 上一帧的状态
          const Eigen::Vector3d &Pwbpre = lidarFrameList->back().P;
          const Eigen::Quaterniond &Qwbpre = lidarFrameList->back().Q;
          const Eigen::Vector3d &Vwbpre = lidarFrameList->back().V;

          // 预积分量
          const Eigen::Quaterniond &dQ = lidarFrame.imuIntegrator.GetDeltaQ();
          const Eigen::Vector3d &dP = lidarFrame.imuIntegrator.GetDeltaP();
          const Eigen::Vector3d &dV = lidarFrame.imuIntegrator.GetDeltaV();
          double dt = lidarFrame.imuIntegrator.GetDeltaTime();

          // 预测当前帧在世界坐标系下的位姿
          lidarFrame.Q = Qwbpre * dQ;
          lidarFrame.P = Pwbpre + Vwbpre * dt + 0.5 * GravityVector * dt * dt + Qwbpre * dP;
          lidarFrame.V = Vwbpre + GravityVector * dt + Qwbpre * dV;
          lidarFrame.bg = lidarFrameList->back().bg;
          lidarFrame.ba = lidarFrameList->back().ba;

          // 计算在 LiDAR 坐标系下的相对位姿变化
          Eigen::Quaterniond Qwlpre = Qwbpre * Eigen::Quaterniond(exRbl);
          Eigen::Vector3d Pwlpre = Qwbpre * exPbl + Pwbpre;

          Eigen::Quaterniond Qwl = lidarFrame.Q * Eigen::Quaterniond(exRbl);
          Eigen::Vector3d Pwl = lidarFrame.Q * exPbl + lidarFrame.P;

          delta_Rl = Qwlpre.conjugate() * Qwl;
          delta_tl = Qwlpre.conjugate() * (Pwl - Pwlpre);
          delta_Rb = dQ.toRotationMatrix();
          delta_tb = dP;

          lidarFrameList->push_back(lidarFrame);
          lidarFrameList->pop_front();
          lidar_list = lidarFrameList;
        }
      }
      else
      {
        // 若未获取IMU消息
        if (LidarIMUInited)
        {
          // 一旦完成初始化后，需要IMU数据来继续
          break;
        }
        else
        {
          // 简单地沿用上一帧的结果进行预测
          lidarFrame.P = transformAftMapped.topLeftCorner(3, 3) * delta_tb + transformAftMapped.topRightCorner(3, 1);
          Eigen::Matrix3d m3d = transformAftMapped.topLeftCorner(3, 3) * delta_Rb;
          lidarFrame.Q = m3d;

          lidar_list.reset(new std::list<Estimator::LidarFrame>);
          lidar_list->push_back(lidarFrame);
        }
      }

      // 4. 去除 LiDAR 畸变
      RemoveLidarDistortion(laserCloudFullRes, delta_Rl, delta_tl);

      // 5. 调用外部 Estimator 模块对当前 LiDAR 帧进行优化
      estimator->EstimateLidarPose(*lidar_list, exTlb, GravityVector, debugInfo);

      // 根据优化后的位姿更新变换矩阵
      Eigen::Matrix4d transformTobeMapped = Eigen::Matrix4d::Identity();
      transformTobeMapped.topLeftCorner(3, 3) = lidar_list->front().Q * exRbl;
      transformTobeMapped.topRightCorner(3, 1) = lidar_list->front().Q * exPbl + lidar_list->front().P;

      // 更新相对位姿增量
      delta_Rb = transformAftMapped.topLeftCorner(3, 3).transpose() * lidar_list->front().Q.toRotationMatrix();
      delta_tb = transformAftMapped.topLeftCorner(3, 3).transpose() * (lidar_list->front().P - transformAftMapped.topRightCorner(3, 1));

      Eigen::Matrix3d Rwlpre = transformAftMapped.topLeftCorner(3, 3) * exRbl;
      Eigen::Vector3d Pwlpre = transformAftMapped.topLeftCorner(3, 3) * exPbl + transformAftMapped.topRightCorner(3, 1);
      delta_Rl = Rwlpre.transpose() * transformTobeMapped.topLeftCorner(3, 3);
      delta_tl = Rwlpre.transpose() * (transformTobeMapped.topRightCorner(3, 1) - Pwlpre);

      transformAftMapped.topLeftCorner(3, 3) = lidar_list->front().Q.toRotationMatrix();
      transformAftMapped.topRightCorner(3, 1) = lidar_list->front().P;

      // 6. 发布优化后的里程计消息
      pubOdometry(transformTobeMapped, lidar_list->front().timeStamp);

      // 7. 将当前帧的点云进行位姿变换后发布
      int laserCloudFullResNum = lidar_list->front().laserCloud->points.size();
      pcl::PointCloud<PointType>::Ptr laserCloudAfterEstimate(new pcl::PointCloud<PointType>());
      laserCloudAfterEstimate->reserve(laserCloudFullResNum);
      for (int i = 0; i < laserCloudFullResNum; i++)
      {
        PointType temp_point;
        MAP_MANAGER::pointAssociateToMap(&lidar_list->front().laserCloud->points[i], &temp_point, transformTobeMapped);
        laserCloudAfterEstimate->push_back(temp_point);
      }
      sensor_msgs::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(*laserCloudAfterEstimate, laserCloudMsg);
      laserCloudMsg.header.frame_id = "/world";
      laserCloudMsg.header.stamp.fromSec(lidar_list->front().timeStamp);
      pubFullLaserCloud.publish(laserCloudMsg);

      // 8. 如果启用紧耦合 IMU 模式，维护 LiDAR 帧队列，并尝试进行初始化
      if (IMU_Mode > 1 && !LidarIMUInited)
      {
        // 更新当前帧 LiDAR 的世界位姿
        lidarFrame.P = transformTobeMapped.topRightCorner(3, 1);
        Eigen::Matrix3d m3d = transformTobeMapped.topLeftCorner(3, 3);
        lidarFrame.Q = m3d;

        // 静态计数 pushCount 用于控制将 LiDAR 帧推入滑窗的频率
        if (pushCount == 0)
        {
          lidarFrameList->push_back(lidarFrame);
          lidarFrameList->back().imuIntegrator.Reset(); // 重置该帧的 IMU 积分器
          if (lidarFrameList->size() > WINDOWSIZE)
            lidarFrameList->pop_front();
        }
        else
        {
          lidarFrameList->back().laserCloud = lidarFrame.laserCloud;
          lidarFrameList->back().imuIntegrator.PushIMUMsg(vimuMsg);
          lidarFrameList->back().timeStamp = lidarFrame.timeStamp;
          lidarFrameList->back().P = lidarFrame.P;
          lidarFrameList->back().Q = lidarFrame.Q;
        }
        pushCount++;

        // 当积累的帧数达到一定数量后，进行一次联合初始化或更新
        if (pushCount >= 3)
        {
          pushCount = 0;
          if (lidarFrameList->size() > 1)
          {
            auto iterRight = std::prev(lidarFrameList->end());
            auto iterLeft = std::prev(std::prev(lidarFrameList->end()));
            iterRight->imuIntegrator.PreIntegration(iterLeft->timeStamp, iterLeft->bg, iterLeft->ba);
          }

          // 记录初始化开始时间
          if (lidarFrameList->size() == int(WINDOWSIZE / 1.5))
          {
            startTime = lidarFrameList->back().timeStamp;
          }

          // 如果帧数达到窗口大小，并且最早的帧时间大于 startTime，进行 MAP 初始化
          if (!LidarIMUInited && lidarFrameList->size() == WINDOWSIZE && lidarFrameList->front().timeStamp >= startTime)
          {
            ROS_INFO("**************Start MAP Initialization!!!******************");
            if (TryMAPInitialization())
            {
              LidarIMUInited = true;
              pushCount = 0;
              startTime = 0;
            }
            ROS_INFO("**************Finish MAP Initialization!!!******************");
          }
        }
      }

      // 记录当前帧时间戳，供下一次使用
      time_last_lidar = time_curr_lidar;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PoseEstimation");
  ros::NodeHandle nodeHandler("~");

  ros::param::get("~filter_parameter_corner", filter_parameter_corner);
  ros::param::get("~filter_parameter_surf", filter_parameter_surf);
  ros::param::get("~IMU_Mode", IMU_Mode);
  std::vector<double> vecTlb;
  ros::param::get("~Extrinsic_Tlb", vecTlb);

  // set extrinsic matrix between lidar & IMU
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  R << vecTlb[0], vecTlb[1], vecTlb[2],
      vecTlb[4], vecTlb[5], vecTlb[6],
      vecTlb[8], vecTlb[9], vecTlb[10];
  t << vecTlb[3], vecTlb[7], vecTlb[11];
  Eigen::Quaterniond qr(R);
  R = qr.normalized().toRotationMatrix();
  exTlb.topLeftCorner(3, 3) = R;
  exTlb.topRightCorner(3, 1) = t;
  exRlb = R;
  exRbl = R.transpose();
  exPlb = t;
  exPbl = -1.0 * exRbl * exPlb;

  ros::Subscriber subFullCloud = nodeHandler.subscribe<sensor_msgs::PointCloud2>("/livox_full_cloud", 10, fullCallBack);
  ros::Subscriber sub_imu;
  if (IMU_Mode > 0)
    sub_imu = nodeHandler.subscribe("/livox/imu", 2000, imu_callback, ros::TransportHints().unreliable());
  if (IMU_Mode < 2)
    WINDOWSIZE = 1;
  else
    WINDOWSIZE = 20;

  pubFullLaserCloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("/livox_full_cloud_mapped", 10);
  pubLaserOdometry = nodeHandler.advertise<nav_msgs::Odometry>("/livox_odometry_mapped", 5);
  pubLaserOdometryPath = nodeHandler.advertise<nav_msgs::Path>("/livox_odometry_path_mapped", 5);
  pubGps = nodeHandler.advertise<sensor_msgs::NavSatFix>("/lidar", 1000);

  tfBroadcaster = new tf::TransformBroadcaster();

  laserCloudFullRes.reset(new pcl::PointCloud<PointType>);
  estimator = new Estimator(filter_parameter_corner, filter_parameter_surf);
  lidarFrameList.reset(new std::list<Estimator::LidarFrame>);

  std::thread thread_process{process};
  ros::spin();

  return 0;
}
