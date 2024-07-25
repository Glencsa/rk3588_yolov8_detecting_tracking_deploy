#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
typedef Matrix<double, 3, 4> Matrix34d;

#pragma pack(1)
struct uav_state
{
    int32_t lng;
    int32_t lat;
    int32_t gps_h;
    int32_t xiangdui_h;
    int32_t qiya_h;
    int16_t north_speed;
    int16_t east_speed;
    int16_t down_speed;
    int16_t roll_uav;
    int16_t pitch_uav;
    int16_t yaw_uav;
    uint8_t state1;
    uint8_t state2;
    int16_t roll_dyt;
    int16_t pitch_dyt;
    int16_t yaw_dyt;
    int16_t roll_speed;
    int16_t pitch_speed;
    int16_t yaw_speed;
    int32_t none;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t t;
    uint8_t m;
    uint8_t s;
    uint8_t jiaoyan;
};
#pragma pack()

// location,定位算法输入参数
struct uav_info
{
    Vector3d xyz_world; // 无人机在世界坐标系下的位置
    double pitch;
    double roll;
    double yaw;
};
struct cam_info
{
    Vector3d xyz_uav; // 相机在无人机坐标系下的位置
    double pitch;
    double roll;
    double yaw;
    Matrix3d K;
};
struct one_info
{
    uav_info uav;
    cam_info cam;
    Vector2d uv;
};

class SingleView
{
    Matrix34d K;
    Matrix4d RT;

public:
    SingleView(Matrix34d _K);
    SingleView(Matrix3d _K);
    ~SingleView() {};
    const Matrix34d &getK();
    const Matrix4d &getRT();
    // R为原坐标系变换到之后坐标系
    void addRotation(Matrix3d R);
    // t为原坐标系变换到之后坐标系
    void addTranslation(Eigen::Vector3d t);

    void addRotationXYZ(float roll, float pitch, float yaw, bool degree);
    Vector2d singleViewLocate(double x, double y, double Z);

    Vector2d world2px(Eigen::Vector4d XYZ_o);
};
class MultiView
{
public:
    std::vector<SingleView> viewList;
    void addView(SingleView sv);
    Vector3d multiViewLocate(std::vector<Vector2d> pixList);
};
