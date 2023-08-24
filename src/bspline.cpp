#include <bspline/bspline.h> 
#include <ros/ros.h>


BSplineGenerator::BSplineGenerator() : degree_(3) {}

void BSplineGenerator::setDegree(int degree) {
    // 阶数必须是大于等于1的整数
    if (degree >= 1) {
        degree_ = degree;
    }
}

void BSplineGenerator::setControlPoints(const std::vector<geometry_msgs::Point>& control_points) {
    control_points_ = control_points;
}

// 计算B样条基函数
double BSplineGenerator::BSplineBasisFunction(int i, int p, double t, const std::vector<double>& knots) {
    if (p == 0) {
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    }

    double basis = 0.0;

    double denominator1 = knots[i + p] - knots[i];
    double denominator2 = knots[i + p + 1] - knots[i + 1];

    if (denominator1 > 0.0) {
        basis += ((t - knots[i]) / denominator1) * BSplineBasisFunction(i, p - 1, t, knots);
    }

    if (denominator2 > 0.0) {
        basis += ((knots[i + p + 1] - t) / denominator2) * BSplineBasisFunction(i + 1, p - 1, t, knots);
    }

    return basis;
}

void BSplineGenerator::generateBSpline(std::vector<geometry_msgs::Point>& bspline_points) {
    if (control_points_.size() < degree_ + 1) {
        ROS_WARN("The control points are less! ");
        return; // 控制点数量不足，无法生成B样条曲线
    }

    bspline_points.clear();

    // 生成均匀节点矢量
    int n = control_points_.size() + degree_ + 1;
    std::vector<double> knots(n);

    double step = 1.0 / (n - 2 * degree_ - 1);
    for (int i = 0; i < n; ++i) {
        if (i < degree_ + 1) {
            knots[i] = 0.0;
        } else if (i > n - degree_ - 1) {
            knots[i] = 1.0;
        } else {
            knots[i] = knots[i - 1] + step;
        }
    }

    // 生成B样条曲线的离散点
    int num_points = 100; // 离散点数量，可以根据需要调整
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        for (size_t j = 0; j < control_points_.size(); ++j) {
            double basis = BSplineBasisFunction(j, degree_, t, knots);
            x += basis * control_points_[j].x;
            y += basis * control_points_[j].y;
            z += basis * control_points_[j].z;
        }

        geometry_msgs::Point bspline_point;
        bspline_point.x = x;
        bspline_point.y = y;
        bspline_point.z = z;
        bspline_points.push_back(bspline_point);
    }
}
