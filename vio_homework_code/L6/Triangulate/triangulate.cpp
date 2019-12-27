//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t): Rwc(R), qwc(R), twc(t) {}

    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};


int main()
{
    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius,
                                            radius * sin(theta),
                                            1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    default_random_engine generator;
    uniform_real_distribution<double> xy_rand(-4, 4.0);
    uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator); 
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);

    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    Eigen::Matrix<double, 14, 4> D;
    for (int i = start_frame_id; i < end_frame_id; ++i) {   // 3~9
        double ui = camera_pose[i].uv[0];
        double vi = camera_pose[i].uv[1];

        // 这里R,t是World->Camera的投影
        Eigen::Matrix<double, 3, 4> Pi;
        Pi.block(0, 0, 3, 3) = camera_pose[i].Rwc.inverse();
        Pi.block(0, 3, 3, 1) = -camera_pose[i].Rwc.inverse()*camera_pose[i].twc;

        int idx = 2 * (i - start_frame_id);
        D.block(idx, 0, 1, 4) = ui * Pi.row(2) - Pi.row(0);
        D.block(idx+1, 0, 1, 4) = vi * Pi.row(2) - Pi.row(1);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D.transpose()*D, Eigen::ComputeThinU | Eigen::ComputeThinV);
    cout << "SVD singular values: \n" << svd.singularValues().transpose() << endl;

    Eigen::Matrix4d U = svd.matrixU();  // 这里 U = V
    P_est = U.col(3).head(3) / U.col(3)[3];
    /* your code end */

    cout << "ground truth: \n" << Pw.transpose() << endl;
    cout << "your result: \n" << P_est.transpose() << endl;
    return 0;
}
