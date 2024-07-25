#include "location.h"

Vector3d MultiView::multiViewLocate(std::vector<Vector2d> pixList)
{
    if (pixList.size() != viewList.size())
    {
        std::cout << "error" << std::endl;
        return Vector3d::Zero();
    }
    int n = pixList.size();
    MatrixXd A(2 * n, 3);
    MatrixXd B(2 * n, 1);
    for (int i = 0; i < n; i++)
    {
        SingleView sv = viewList[i];
        Vector2d p = pixList[i];
        double x = p(0);
        double y = p(1);
        Matrix34d r = sv.getK() * sv.getRT();
        A.row(2 * i) << x * r(2, 0) - r(0, 0), x * r(2, 1) - r(0, 1), x * r(2, 2) - r(0, 2);
        A.row(2 * i + 1) << y * r(2, 0) - r(1, 0), y * r(2, 1) - r(1, 1), y * r(2, 2) - r(1, 2);
        B(2 * i, 0) = -x * r(2, 3) + r(0, 3);
        B(2 * i + 1, 0) = -y * r(2, 3) + r(1, 3);
    }
    Vector3d x = (A.transpose() * A).inverse() * (A.transpose()) * B;
    return x;
};

void MultiView::addView(SingleView sv)
{
    viewList.push_back(sv);
};
SingleView::SingleView(Matrix34d _K)
{
    K = _K;
    RT = Matrix4d::Identity();
}
SingleView::SingleView(Matrix3d _K)
{
    K.block<3, 3>(0, 0) = _K;
    K.col(3).setZero();
    RT = Matrix4d::Identity();
}
const Matrix34d &SingleView::getK()
{
    return K;
}
const Matrix4d &SingleView::getRT()
{
    return RT;
}
// R为原坐标系变换到之后坐标系
void SingleView::addRotation(Matrix3d R)
{
    Matrix4d P1 = Matrix4d::Identity();
    P1.block<3, 3>(0, 0) = R.transpose();
    RT = P1 * RT;
}
// t为原坐标系变换到之后坐标系
void SingleView::addTranslation(Eigen::Vector3d t)
{
    Matrix4d P1 = Matrix4d::Identity();
    P1.block<3, 1>(0, 3) = -t;
    RT = P1 * RT;
}

void SingleView::addRotationXYZ(float roll, float pitch, float yaw, bool degree = false)
{
    if (degree)
    {
        pitch = pitch * M_PI / 180;
        roll = roll * M_PI / 180;
        yaw = yaw * M_PI / 180;
    }
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d r = q.matrix();

    addRotation(r);
}
Vector2d SingleView::singleViewLocate(double x, double y, double Z)
{
    Matrix34d r = K * RT;
    Matrix2d A;
    A << x * r(2, 0) - r(0, 0), x * r(2, 1) - r(0, 1),
        y * r(2, 0) - r(1, 0), y * r(2, 1) - r(1, 1);
    Vector2d B;
    B << -(x * r(2, 2) - r(0, 2)) * Z - x * r(2, 3) + r(0, 3),
        -(y * r(2, 2) - r(1, 2)) * Z - y * r(2, 3) + r(1, 3);
    Vector2d result = A.colPivHouseholderQr().solve(B);
    return result;
}

Vector2d SingleView::world2px(Eigen::Vector4d XYZ_o)
{
    Matrix34d P1 = K * RT;
    Vector3d uv = P1 * XYZ_o;
    uv /= uv(2);
    return uv.head<2>();
}
