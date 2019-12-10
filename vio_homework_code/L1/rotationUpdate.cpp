#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <sophus/so3.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    // 生成一个旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3<double> SO3_R(R);
    cout << endl << "Generate a rotation matrix R:" << endl << R << endl;
    cout << endl << "R to so3: " << SO3_R.log().transpose() << endl;

    // 微小变化
    Eigen::Vector3d w;
    w << 0.01, 0.02, 0.03;
    cout << endl << "With a small change w: " << w.transpose() << endl;

    // 两种微小变化的矩阵形式
    Eigen::Matrix3d delta_R = Sophus::SO3<double>::exp(w).matrix();
    cout << endl << "Small change w to exp(w^):" << endl << delta_R << endl;
    Eigen::Quaternion<double> delta_q(1, w[0] / 2, w[1] / 2, w[2] / 2);
    delta_q.normalize();
    cout << endl << "Small change w to quaternion(1, w/2) to rotation matrix:" << endl
         << delta_q.toRotationMatrix() << endl;

    // R*exp(w^)更新结果
    Sophus::SO3<double> SO3_R1 = SO3_R * Sophus::SO3<double>::exp(w);
    cout << endl << "Update rotation matrix R with R*exp(w^):" << endl << SO3_R1.matrix() << endl;
    cout << endl << "Updated R to so3: " << SO3_R1.log().transpose() << endl;

    // q*[1, w/2]更新结果
    Eigen::Quaternion<double> q(R);
    Eigen::Quaternion<double> q1 = q * delta_q;
    q1.normalize();
    Eigen::Matrix3d R2 = q1.toRotationMatrix();
    cout << endl << "Update rotation matrix R with q*[1, w/2]:" << endl << R2 << endl;
    cout << endl << "Updated R to so3: " << Sophus::SO3<double>(R2).log().transpose() << endl;

    return 0;
}
