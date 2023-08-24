#ifndef BSPLINE_H
#define BSPLINE_H

#include <geometry_msgs/Point.h>

class BSplineGenerator {
public:
    BSplineGenerator();

    void setDegree(int degree);
    void setControlPoints(const std::vector<geometry_msgs::Point>& control_points);
    void generateBSpline(std::vector<geometry_msgs::Point>& bspline_points);

private:
    int degree_; // B样条的阶数
    std::vector<geometry_msgs::Point> control_points_; // 控制点

    // 计算B样条基函数
    double BSplineBasisFunction(int i, int p, double t, const std::vector<double>& knots);
};

#endif // BSPLINE_H
