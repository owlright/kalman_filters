#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>

using namespace Eigen;
using namespace std;

struct Pose {
    Matrix3d R; // 旋转
    Vector3d t; // 平移
};

vector<Pose> generateSphericalTrajectory(int num_poses, double radius, double dt) {
    vector<Pose> poses;
    // 初始位姿：北极点，朝向z轴
    Vector3d position(0, 0, radius);
    Matrix3d rotation = Matrix3d::Identity();

    // 运动参数
    double theta_rate = M_PI / (num_poses * dt); // 极角速度
    double phi_rate = 2 * M_PI / (num_poses * dt); // 方位角速度

    double theta = 0.0;
    double phi = 0.0;

    for (int i = 0; i < num_poses; ++i) {
        // 计算当前位置
        position = Vector3d(
            radius * sin(theta) * cos(phi),
            radius * sin(theta) * sin(phi),
            radius * cos(theta)
        );

        // 计算朝向球心的旋转
        Vector3d z_axis = -position.normalized();
        Vector3d x_axis = Vector3d::UnitX();
        if (fabs(z_axis.dot(x_axis)) > 0.99) x_axis = Vector3d::UnitY(); // 防止共线
        Vector3d y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized();
        rotation.col(0) = x_axis;
        rotation.col(1) = y_axis;
        rotation.col(2) = z_axis;

        poses.push_back({rotation, position});

        // 积分运动
        theta += theta_rate * dt;
        phi += phi_rate * dt;
    }
    return poses;
}

int main() {
    int num_poses = 20;
    double radius = 10.0;
    double dt = 0.1;
    auto poses = generateSphericalTrajectory(num_poses, radius, dt);
    std::string filename = "generated_poses.txt";
    std::ofstream file(filename);
    for (size_t i = 0; i < poses.size(); ++i) {
        file << poses[i].t.y() << " " << poses[i].t.z() << " " << poses[i].t.x() << " "
             << i << " ";
        // 计算欧拉角（roll, pitch, yaw）
        double roll = atan2(poses[i].R(2,1), poses[i].R(2,2));
        double pitch = asin(-poses[i].R(2,0));
        double yaw = atan2(poses[i].R(1,0), poses[i].R(0,0));
        file << roll << " " << yaw << " " << pitch << " " << 0.0 << endl;
    }
    file.close();
    return 0;
}