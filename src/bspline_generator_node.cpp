#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <bspline/bspline.h> 
#include <nurbs/nurbs.h>
// 这里的 "bspline.h" 是你实现 B 样条生成算法的头文件，需要与源文件在同一个目录下或正确包含路径

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bspline_generator_node");
    ros::NodeHandle nh;

    // 创建发布器，发布B样条的控制点
    ros::Publisher control_points_pub = nh.advertise<visualization_msgs::Marker>("bspline_control_points", 10);
    ros::Publisher bspline_points_pub = nh.advertise<visualization_msgs::Marker>("bspline_points", 10);


    // 假设你已经有了一组控制点，以 std::vector<geometry_msgs::Point> 的形式存储
    std::vector<geometry_msgs::Point> control_points;
    // ... 在这里填入你的控制点数据 ...
    // 控制点数据
    // std::vector<geometry_msgs::Point> control_points;
    geometry_msgs::Point point;

    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    control_points.push_back(point);

    point.x = 1.0;
    point.y = 2.0;
    point.z = 0.0;
    control_points.push_back(point);

    point.x = 3.0;
    point.y = 1.0;
    point.z = 0.0;
    control_points.push_back(point);

    point.x = 4.0;
    point.y = -1.0;
    point.z = 0.0;
    control_points.push_back(point);

    point.x = 5.0;
    point.y = 0.0;
    point.z = 0.0;
    control_points.push_back(point);
    std::vector<double> weights;
    for(int k = 0;k<5;++k){
        double wei = 1;
        weights.push_back(wei);
    }

    // 创建一个B样条生成器对象
    NURBSGenerator bspline_generator;

    // 设置B样条的阶数和控制点
    bspline_generator.setDegree(3); // 假设阶数为3
    bspline_generator.setControlPoints(control_points,weights);

    // 生成B样条曲线的离散点
    std::vector<geometry_msgs::Point> bspline_points;
    bspline_generator.generateNURBS(bspline_points);

    // 发布控制点
    visualization_msgs::Marker control_points_marker;
    control_points_marker.header.frame_id = "map"; // 假设使用map坐标系
    control_points_marker.type = visualization_msgs::Marker::POINTS;
    control_points_marker.action = visualization_msgs::Marker::ADD;
    control_points_marker.scale.x = control_points_marker.scale.y = 0.2;
    control_points_marker.color.r = 1.0;
    control_points_marker.color.a = 1.0;
    control_points_marker.points = control_points;
    control_points_pub.publish(control_points_marker);

    visualization_msgs::Marker bspline_points_marker;
    bspline_points_marker.header.frame_id = "map"; // 假设使用map坐标系
    bspline_points_marker.type = visualization_msgs::Marker::POINTS;
    bspline_points_marker.action = visualization_msgs::Marker::ADD;
    bspline_points_marker.scale.x = bspline_points_marker.scale.y = 0.2;
    bspline_points_marker.color.g = 1.0;
    bspline_points_marker.color.a = 1.0;
    bspline_points_marker.points = bspline_points;
    // control_points_pub.publish(bspline_points_marker);

    // 循环等待ROS事件
    // ros::spin();
    ros::Rate r(1);
    while(ros::ok()){
        bspline_points_pub.publish(bspline_points_marker);
        control_points_pub.publish(control_points_marker);
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
