#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

using namespace Eigen;
using namespace std;

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv) {
    // std::cout << "Pangolin version: " << PANGOLIN_VERSION_STRING << std::endl;
    string trajectory_file = "./pose.txt";
    if (argc > 1) {
        trajectory_file = string(argv[1]);
    }
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(trajectory_file);
    if (!fin) {
        cout << "cannot find trajectory file at " << trajectory_file << endl;
        return 1;
    }

    // while (!fin.eof()) {
    //     double time, tx, ty, tz, qx, qy, qz, qw;
    //     fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    //     Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    //     Twr.pretranslate(Vector3d(tx, ty, tz));
    //     poses.push_back(Twr);
    // }
    while (!fin.eof())
    {
        double time, intensity, tx, ty, tz, roll, pitch, yaw;
        fin >> ty >> tz >> tx >> intensity >> roll >> yaw >> pitch >> time;
        if (fin.fail()) {

            break; // 如果读取失败（如空行），跳出循环
        }
        AngleAxisd rollAngle(roll, Vector3d::UnitX());
        AngleAxisd pitchAngle(pitch, Vector3d::UnitY());
        AngleAxisd yawAngle(yaw, Vector3d::UnitZ());
        Matrix3d R = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();
        Isometry3d Twr = Isometry3d::Identity();
        Twr.rotate(R);
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    cout << "read total " << poses.size() << " pose entries" << endl;

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

void drawAxes(const Isometry3d& pose, double axis_length = 0.1, float line_width = 2.0f) {
    // 保存当前线宽
    GLfloat prev_width;
    glGetFloatv(GL_LINE_WIDTH, &prev_width);

    glLineWidth(line_width); // 设置新线宽
    // 画每个位姿的三个坐标轴
    Vector3d Ow = pose.translation();
    Vector3d Xw = pose * (axis_length * Vector3d(1, 0, 0));
    Vector3d Yw = pose * (axis_length * Vector3d(0, 1, 0));
    Vector3d Zw = pose * (axis_length * Vector3d(0, 0, 1));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0); // X轴红色
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0); // Y轴绿色
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0); // Z轴蓝色
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Zw[0], Zw[1], Zw[2]);
    glEnd();
    glLineWidth(prev_width); // 恢复线宽
}

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(-0.1, 0.1, 0, 0, 0, 0, 0.0, 0.0, 1.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    const double refreshHz = 60;
    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glLineWidth(2);
        // 显示世界坐标系OpenGlMatrix ModelViewLookAt(GLprecision ex, GLprecision ey, GLprecision ez, GLprecision lx, GLprecision ly, GLprecision lz, GLprecision ux, GLprecision uy, GLprecision uz);
        drawAxes(Isometry3d::Identity(), 1);
        for (size_t i = 0; i < poses.size(); i++) {
            drawAxes(poses[i]);
            // 在每个位姿旁边画序号
            // glColor3f(1.0, 1.0, 1.0);
            // Vector3d pos = poses[i].translation();
            // pangolin::GlText text = pangolin::GlFont::I().Text("%d", int(i));
            // text.Draw(pos[0], pos[1], pos[2]);
        }
        // 画出连线
        // for (size_t i = 0; i < poses.size(); i++) {
        //     glColor3f(0.0, 0.0, 0.0);
        //     glBegin(GL_LINES);
        //     auto p1 = poses[i], p2 = poses[i+1];
        //     glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        //     glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        //     glEnd();
        // }
        pangolin::FinishFrame();
        usleep(1000000/refreshHz);
    }
}