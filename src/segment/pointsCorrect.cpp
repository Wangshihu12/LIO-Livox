#include "segment/pointsCorrect.hpp"

float gnd_pos[6];
int frame_count = 0;
int frame_lenth_threshold = 5; // 5 frames update

/**
 * @brief 获取点云的主方向和邻域点的信息
 *
 * @param npca 存储PCA结果和邻域点信息的结构体
 * @param cloud 输入点云
 * @param kdtree kd树，用于快速邻域搜索
 * @param searchPoint 搜索中心点
 * @param fSearchRadius 搜索半径
 * @return 返回邻域点的数量
 */
int GetNeiborPCA_cor(SNeiborPCA_cor &npca, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZ> kdtree, pcl::PointXYZ searchPoint, float fSearchRadius)
{
    // 存储搜索到的点的距离和邻域点云
    std::vector<float> k_dis;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 如果搜索到的点数大于5，则进行主成分分析
    if (kdtree.radiusSearch(searchPoint, fSearchRadius, npca.neibors, k_dis) > 5)
    {
        // 初始化邻域点云大小
        subCloud->width = npca.neibors.size();
        subCloud->height = 1;
        subCloud->points.resize(subCloud->width * subCloud->height);

        // 遍历邻域点，将点云添加到subCloud中
        for (int pid = 0; pid < subCloud->points.size(); pid++)
        {
            subCloud->points[pid].x = cloud->points[npca.neibors[pid]].x;
            subCloud->points[pid].y = cloud->points[npca.neibors[pid]].y;
            subCloud->points[pid].z = cloud->points[npca.neibors[pid]].z;
        }

        // 利用PCA主元分析法获得点云的三个主方向
        Eigen::Vector4f pcaCentroid;                                                // 点云质心
        pcl::compute3DCentroid(*subCloud, pcaCentroid);                             // 计算质心
        Eigen::Matrix3f covariance;                                                 // 协方差矩阵
        pcl::computeCovarianceMatrixNormalized(*subCloud, pcaCentroid, covariance); // 计算协方差矩阵

        // 求协方差矩阵的特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        npca.eigenVectorsPCA = eigen_solver.eigenvectors(); // 主方向
        npca.eigenValuesPCA = eigen_solver.eigenvalues();   // 主方向对应的特征值

        // 对特征值进行归一化
        float vsum = npca.eigenValuesPCA(0) + npca.eigenValuesPCA(1) + npca.eigenValuesPCA(2);
        npca.eigenValuesPCA(0) = npca.eigenValuesPCA(0) / (vsum + 0.000001);
        npca.eigenValuesPCA(1) = npca.eigenValuesPCA(1) / (vsum + 0.000001);
        npca.eigenValuesPCA(2) = npca.eigenValuesPCA(2) / (vsum + 0.000001);
    }
    else
    {
        // 如果邻域点数不足5个，则清空邻域点
        npca.neibors.clear();
    }

    // 返回邻域点数量
    return npca.neibors.size();
}

/**
 * @brief 筛选点云中的地面点，用于地面检测和矫正
 *
 * @param[out] outPoints 输出的地面点云数据
 * @param[in] inPoints 输入的点云数据
 * @param[in] inNum 输入点云的点数
 * @return 筛选出的地面点数量
 */
int FilterGndForPos_cor(float *outPoints, float *inPoints, int inNum)
{
    // 初始化变量
    int outNum = 0;               // 输出地面点数量
    float dx = 2, dy = 2;         // 栅格的分辨率
    int x_len = 20, y_len = 10;   // 栅格范围
    int nx = 2 * x_len / dx;      // 栅格在x方向的数量
    int ny = 2 * y_len / dy;      // 栅格在y方向的数量
    float offx = -20, offy = -10; // 栅格偏移量
    float THR = 0.4;              // 高度差阈值

    // 为栅格相关的变量分配内存
    float *imgMinZ = (float *)calloc(nx * ny, sizeof(float));  // 栅格内最小高度
    float *imgMaxZ = (float *)calloc(nx * ny, sizeof(float));  // 栅格内最大高度
    float *imgSumZ = (float *)calloc(nx * ny, sizeof(float));  // 栅格内高度和
    float *imgMeanZ = (float *)calloc(nx * ny, sizeof(float)); // 栅格内高度均值
    int *imgNumZ = (int *)calloc(nx * ny, sizeof(int));        // 栅格内点数
    int *idtemp = (int *)calloc(inNum, sizeof(int));           // 点与栅格的索引映射

    // 初始化栅格变量
    for (int ii = 0; ii < nx * ny; ii++)
    {
        imgMinZ[ii] = 10;  // 初始化最小高度为较大值
        imgMaxZ[ii] = -10; // 初始化最大高度为较小值
        imgMeanZ[ii] = -10;
        imgSumZ[ii] = 0;
        imgNumZ[ii] = 0;
    }

    // 遍历输入点云，将点分配到对应的栅格中，并计算高度信息
    for (int pid = 0; pid < inNum; pid++)
    {
        idtemp[pid] = -1;
        // 检查点是否在栅格范围内
        if ((inPoints[pid * 4] > -x_len) && (inPoints[pid * 4] < x_len) &&
            (inPoints[pid * 4 + 1] > -y_len) && (inPoints[pid * 4 + 1] < y_len))
        {
            // 计算点所在的栅格索引
            int idx = (inPoints[pid * 4] - offx) / dx;
            int idy = (inPoints[pid * 4 + 1] - offy) / dy;
            idtemp[pid] = idx + idy * nx;
            if (idtemp[pid] >= nx * ny) // 超出栅格范围则跳过
                continue;

            // 更新栅格的高度统计信息
            imgSumZ[idtemp[pid]] += inPoints[pid * 4 + 2];
            imgNumZ[idtemp[pid]] += 1;
            if (inPoints[pid * 4 + 2] < imgMinZ[idtemp[pid]])
            {
                imgMinZ[idtemp[pid]] = inPoints[pid * 4 + 2];
            }
            if (inPoints[pid * 4 + 2] > imgMaxZ[idtemp[pid]])
            {
                imgMaxZ[idtemp[pid]] = inPoints[pid * 4 + 2];
            }
        }
    }

    // 遍历输入点云，筛选符合地面条件的点
    for (int pid = 0; pid < inNum; pid++)
    {
        if (outNum >= 60000) // 限制最大地面点数
            break;
        if (idtemp[pid] > 0 && idtemp[pid] < nx * ny)
        {
            // 计算栅格的高度均值
            imgMeanZ[idtemp[pid]] = float(imgSumZ[idtemp[pid]] / (imgNumZ[idtemp[pid]] + 0.0001));
            // 筛选条件：高度差小于阈值、点数足够多、均值高度小于2
            if ((imgMaxZ[idtemp[pid]] - imgMeanZ[idtemp[pid]]) < THR &&
                imgNumZ[idtemp[pid]] > 3 && imgMeanZ[idtemp[pid]] < 2)
            {
                // 将符合条件的点加入输出点云
                outPoints[outNum * 4] = inPoints[pid * 4];
                outPoints[outNum * 4 + 1] = inPoints[pid * 4 + 1];
                outPoints[outNum * 4 + 2] = inPoints[pid * 4 + 2];
                outPoints[outNum * 4 + 3] = inPoints[pid * 4 + 3];
                outNum++;
            }
        }
    }

    // 释放分配的内存
    free(imgMinZ);
    free(imgMaxZ);
    free(imgSumZ);
    free(imgMeanZ);
    free(imgNumZ);
    free(idtemp);

    // 返回地面点数量
    return outNum;
}

/**
 * @brief 计算点云的地面法向量和地面中心点坐标
 *
 * @param[out] gnd 输出地面的法向量和中心点坐标 (大小为6的数组：[法向量x, 法向量y, 法向量z, 中心点x, 中心点y, 中心点z])
 * @param[in] fPoints 输入的点云数据 (每个点包含 [x, y, z, intensity])
 * @param[in] pointNum 输入点云的点数
 * @param[in] fSearchRadius 搜索半径，用于PCA分析邻域
 * @return 参与计算的地面点数
 */
int CalGndPos_cor(float *gnd, float *fPoints, int pointNum, float fSearchRadius)
{
    // 初始化地面法向量和中心点坐标
    for (int ii = 0; ii < 6; ii++)
    {
        gnd[ii] = 0;
    }

    // 检查点数是否足够进行地面检测
    if (pointNum <= 3)
    {
        return 0;
    }

    // 将输入点云转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = pointNum;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    for (int pid = 0; pid < cloud->points.size(); pid++)
    {
        cloud->points[pid].x = fPoints[pid * 4];
        cloud->points[pid].y = fPoints[pid * 4 + 1];
        cloud->points[pid].z = fPoints[pid * 4 + 2];
    }

    // 构建KD树，用于快速查找邻域点
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    int nNum = 0;                                                                     // 统计参与地面检测的点数
    unsigned char *pLabel = (unsigned char *)calloc(pointNum, sizeof(unsigned char)); // 点是否已被处理的标记数组

    // 遍历所有点
    for (int pid = 0; pid < pointNum; pid++)
    {
        if ((nNum < 1000) && (pLabel[pid] == 0)) // 限制地面点的最大数量，未处理的点才继续
        {
            SNeiborPCA_cor npca; // 存储PCA分析结果
            pcl::PointXYZ searchPoint;
            searchPoint.x = cloud->points[pid].x;
            searchPoint.y = cloud->points[pid].y;
            searchPoint.z = cloud->points[pid].z;

            // 获取点的邻域PCA分析结果
            if (GetNeiborPCA_cor(npca, cloud, kdtree, searchPoint, fSearchRadius) > 0)
            {
                // 标记邻域点为已处理
                for (int ii = 0; ii < npca.neibors.size(); ii++)
                {
                    pLabel[npca.neibors[ii]] = 1;
                }

                // 检查点的主方向和次方向差异，判断是否为平面点
                if (npca.eigenValuesPCA[1] / (npca.eigenValuesPCA[0] + 0.00001) > 5000)
                {
                    // 确定法向量的方向
                    if (npca.eigenVectorsPCA(2, 0) > 0) // 垂直向上
                    {
                        gnd[0] += npca.eigenVectorsPCA(0, 0);
                        gnd[1] += npca.eigenVectorsPCA(1, 0);
                        gnd[2] += npca.eigenVectorsPCA(2, 0);
                    }
                    else // 垂直向下
                    {
                        gnd[0] -= npca.eigenVectorsPCA(0, 0);
                        gnd[1] -= npca.eigenVectorsPCA(1, 0);
                        gnd[2] -= npca.eigenVectorsPCA(2, 0);
                    }

                    // 累加地面中心点坐标
                    gnd[3] += searchPoint.x;
                    gnd[4] += searchPoint.y;
                    gnd[5] += searchPoint.z;

                    nNum++; // 增加有效点计数
                }
            }
        }
    }

    // 释放内存
    free(pLabel);

    // 如果找到地面点，计算平均法向量和中心点
    if (nNum > 0)
    {
        for (int ii = 0; ii < 6; ii++)
        {
            gnd[ii] /= nNum; // 平均法向量 & 中心点
        }

        // 调整法向量权重，增强地面方向特征
        if (abs(gnd[0]) < 0.1)
        {
            gnd[0] = gnd[0] * (1 - abs(gnd[0])) * 4.5;
        }
        else if (abs(gnd[0]) < 0.2)
        {
            gnd[0] = gnd[0] * (1 - abs(gnd[0])) * 3.2;
        }
        else
        {
            gnd[0] = gnd[0] * (1 - abs(gnd[0])) * 2.8;
        }
        gnd[1] = gnd[1] * 2.3;
    }

    // 返回参与地面检测的点数
    return nNum;
}

/**
 * @brief 根据两个向量计算旋转矩阵，使一个向量旋转到另一个向量
 *
 * @param[out] RTM 输出旋转矩阵 (大小为3x3的一维数组)
 * @param[in] v0 输入的初始向量，旋转前的向量 (大小为3)
 * @param[in] v1 输入的目标向量，旋转后的向量 (大小为3)
 * @return 返回值为0，表示计算成功
 */
int GetRTMatrix_cor(float *RTM, float *v0, float *v1)
{
    // **1. 向量归一化**
    // 计算v0的模长，并对其进行归一化
    float nv0 = sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]);
    v0[0] /= (nv0 + 0.000001); // 防止除0
    v0[1] /= (nv0 + 0.000001);
    v0[2] /= (nv0 + 0.000001);

    // 计算v1的模长，并对其进行归一化
    float nv1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
    v1[0] /= (nv1 + 0.000001);
    v1[1] /= (nv1 + 0.000001);
    v1[2] /= (nv1 + 0.000001);

    // **2. 计算两个向量的叉乘**
    // 叉乘结果是旋转轴方向向量
    float v2[3];
    v2[0] = v0[1] * v1[2] - v0[2] * v1[1]; // x分量
    v2[1] = v0[2] * v1[0] - v0[0] * v1[2]; // y分量
    v2[2] = v0[0] * v1[1] - v0[1] * v1[0]; // z分量

    // **3. 计算夹角的余弦和正弦值**
    // 两向量的点乘得到余弦值
    float cosAng = v0[0] * v1[0] + v0[1] * v1[1] + v0[2] * v1[2];
    // 根据余弦值计算正弦值
    float sinAng = sqrt(1 - cosAng * cosAng);

    // **4. 构造旋转矩阵**
    // 使用罗德里格斯旋转公式计算旋转矩阵
    RTM[0] = v2[0] * v2[0] * (1 - cosAng) + cosAng;
    RTM[4] = v2[1] * v2[1] * (1 - cosAng) + cosAng;
    RTM[8] = v2[2] * v2[2] * (1 - cosAng) + cosAng;

    RTM[1] = RTM[3] = v2[0] * v2[1] * (1 - cosAng);
    RTM[2] = RTM[6] = v2[0] * v2[2] * (1 - cosAng);
    RTM[5] = RTM[7] = v2[1] * v2[2] * (1 - cosAng);

    // 添加正弦项，处理旋转方向
    RTM[1] += (v2[2]) * sinAng;
    RTM[2] += (-v2[1]) * sinAng;
    RTM[3] += (-v2[2]) * sinAng;

    RTM[5] += (v2[0]) * sinAng;
    RTM[6] += (v2[1]) * sinAng;
    RTM[7] += (-v2[0]) * sinAng;

    return 0; // 计算成功
}

/**
 * @brief 对点云进行地面矫正，将点云中的地面校正到水平面上
 *
 * @param[in,out] fPoints 输入和输出的点云数据，每个点由4个浮点数表示（x, y, z, intensity）
 * @param[in] pointNum 点云中的点数
 * @param[in] gndPos 地面的平面信息，包括法向量 (nx, ny, nz) 和一个地面点 (x, y, z)，大小为6的数组
 * @return 返回值为0，表示矫正完成
 */
int CorrectPoints_cor(float *fPoints, int pointNum, float *gndPos)
{
    // **1. 计算旋转矩阵**
    // RTM：旋转矩阵 (3x3)，将地面法向量 (gndPos[0], gndPos[1], gndPos[2]) 矫正到 z 轴方向 [0, 0, 1]
    float RTM[9];
    // znorm：目标法向量，表示水平面的法向量
    float znorm[3] = {0, 0, 1};
    // 调用函数计算旋转矩阵 RTM
    GetRTMatrix_cor(RTM, gndPos, znorm);

    // **2. 计算地面高度**
    // 地面高度 gndHeight 表示旋转到水平面后地面的 z 值
    float gndHeight = RTM[2] * gndPos[3] + RTM[5] * gndPos[4] + RTM[8] * gndPos[5];

    // **3. 对每个点进行矫正**
    for (int pid = 0; pid < pointNum; pid++)
    {
        // tmp 用于存储旋转后的点坐标
        float tmp[3];

        // 旋转点云：计算旋转后的 (x, y, z)
        tmp[0] = RTM[0] * fPoints[pid * 4] + RTM[3] * fPoints[pid * 4 + 1] + RTM[6] * fPoints[pid * 4 + 2];
        tmp[1] = RTM[1] * fPoints[pid * 4] + RTM[4] * fPoints[pid * 4 + 1] + RTM[7] * fPoints[pid * 4 + 2];
        tmp[2] = RTM[2] * fPoints[pid * 4] + RTM[5] * fPoints[pid * 4 + 1] + RTM[8] * fPoints[pid * 4 + 2] - gndHeight;

        // 更新矫正后的点坐标
        fPoints[pid * 4] = tmp[0];     // 更新 x 坐标
        fPoints[pid * 4 + 1] = tmp[1]; // 更新 y 坐标
        fPoints[pid * 4 + 2] = tmp[2]; // 更新 z 坐标
    }

    // **4. 返回完成状态**
    return 0; // 矫正完成
}

/**
 * @brief 获取地面位置和法向量，结合当前帧地面点数据和历史地面估计更新结果。
 *
 * @param pos 输出参数，用于存储更新后的地面搜索点和法向量（长度为6的数组）。
 * @param fPoints 输入的点云数据，每个点包含x、y、z和强度信息。
 * @param pointNum 输入点云的总点数。
 * @return 返回值始终为0。
 */
int GetGndPos(float *pos, float *fPoints, int pointNum)
{
    // 动态分配存储过滤后的地面点的数组
    float *fPoints3 = (float *)calloc(60000 * 4, sizeof(float));
    // 筛选地面点，返回筛选后的点数
    int pnum3 = FilterGndForPos_cor(fPoints3, fPoints, pointNum);

    // 如果地面点数过少，输出警告信息
    if (pnum3 < 3)
    {
        std::cout << "too few ground points!\n";
    }

    // 临时数组，用于存储本帧计算的地面参数（法向量和地面搜索点）
    float tmpPos[6];
    // 调用地面参数计算函数，获取地面法向量和地面搜索点，存储在 tmpPos 中
    int gndnum = CalGndPos_cor(tmpPos, fPoints3, pnum3, 1.0);

    // 如果全局地面参数尚未初始化（`gnd_pos[5] == 0`），直接用本帧结果初始化
    if (gnd_pos[5] == 0)
    {
        memcpy(gnd_pos, tmpPos, sizeof(tmpPos));
    }
    else
    {
        // 如果全局地面参数已经初始化，进行更新
        if (frame_count < frame_lenth_threshold && tmpPos[5] != 0)
        {
            // 如果地面点数足够且当前帧法向量与全局法向量的差异较小，则更新地面参数
            if (gndnum > 0 && abs(gnd_pos[0] - tmpPos[0]) < 0.1 && abs(gnd_pos[1] - tmpPos[1]) < 0.1)
            {
                // 通过加权平均更新全局地面参数
                for (int i = 0; i < 6; i++)
                {
                    gnd_pos[i] = (gnd_pos[i] + tmpPos[i]) * 0.5;
                }
                frame_count = 0; // 重置帧计数
            }
            else
            {
                // 如果差异较大，则增加帧计数
                frame_count++;
            }
        }
        else if (tmpPos[5] != 0)
        {
            // 如果超出帧计数阈值，则直接用当前帧的地面参数替换全局参数
            memcpy(gnd_pos, tmpPos, sizeof(tmpPos));
            frame_count = 0; // 重置帧计数
        }
    }

    // 将更新后的地面参数复制到输出参数 `pos`
    memcpy(pos, gnd_pos, sizeof(float) * 6);

    // 释放动态分配的内存
    free(fPoints3);

    return 0;
}
