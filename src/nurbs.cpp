#include <nurbs/nurbs.h>


NURBSGenerator::NURBSGenerator() : degree_(3) {}

void NURBSGenerator::setDegree(int degree) {
    // 阶数必须是大于等于1的整数
    if (degree >= 1) {
        degree_ = degree;
    }
}

void NURBSGenerator::setControlPoints(const std::vector<geometry_msgs::Point>& control_points, const std::vector<double>& weights) {
    control_points_ = control_points;
    weights_ = weights;
}

// 计算NURBS曲线的基函数
double NURBSGenerator::NURBSBasisFunction(int i, int p, double t, const std::vector<double>& knots) {
    if (p == 0) {
        return (knots[i] <= t && t < knots[i + 1]) ? 1.0 : 0.0;
    }

    double denominator1 = knots[i + p] - knots[i];
    double denominator2 = knots[i + p + 1] - knots[i + 1];

    double term1 = 0.0;
    double term2 = 0.0;

    if (denominator1 > 0.0) {
        term1 = ((t - knots[i]) / denominator1) * NURBSBasisFunction(i, p - 1, t, knots);
    }

    if (denominator2 > 0.0) {
        term2 = ((knots[i + p + 1] - t) / denominator2) * NURBSBasisFunction(i + 1, p - 1, t, knots);
    }

    return term1 + term2;
}

void NURBSGenerator::generateNURBS(std::vector<geometry_msgs::Point>& nurbs_points) {
    if (control_points_.size() < degree_ + 1 || control_points_.size() != weights_.size()) {
        return; // 控制点数量不足或权重数量不匹配，无法生成NURBS曲线
    }

    nurbs_points.clear();

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

    // 生成NURBS曲线的离散点
    int num_points = 100; // 离散点数量，可以根据需要调整
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        for (size_t j = 0; j < control_points_.size(); ++j) {
            double basis = NURBSBasisFunction(j, degree_, t, knots) * weights_[j];
            x += basis * control_points_[j].x;
            y += basis * control_points_[j].y;
            z += basis * control_points_[j].z;
        }

        geometry_msgs::Point nurbs_point;
        nurbs_point.x = x;
        nurbs_point.y = y;
        nurbs_point.z = z;
        nurbs_points.push_back(nurbs_point);
    }
}
