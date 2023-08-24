#ifndef NURBS_H
#define NURBS_H

// #include <vector>
#include <geometry_msgs/Point.h>

class NURBSGenerator {
public:
    NURBSGenerator();

    void setDegree(int degree);
    void setControlPoints(const std::vector<geometry_msgs::Point>& control_points, const std::vector<double>& weights);
    void generateNURBS(std::vector<geometry_msgs::Point>& nurbs_points);

private:
    int degree_; // NURBS曲线的阶数
    std::vector<geometry_msgs::Point> control_points_; // 控制点
    std::vector<double> weights_; // 权重
    // ... 其他私有成员变量 ...

    // 计算NURBS曲线的基函数
    double NURBSBasisFunction(int i, int p, double t, const std::vector<double>& knots);
};

#endif // NURBS_H
