//
// Created by tioeare on 5/9/23.
//

#ifndef DATA_FUSION_FILTER_H
#define DATA_FUSION_FILTER_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fusion {

class filter {
public:
    void update(std::vector<float> &data, const float &dtime, const Eigen::MatrixXf &control_H,
                std::vector<double> &new_data);

    explicit filter(float dtime);

    ~filter() = default;

private:
    Eigen::VectorXf hat_x;      // 后验位移和速度
    Eigen::VectorXf measure_x;  // 观测位移和速度
    Eigen::MatrixXf control_A;
    Eigen::MatrixXf control_H;
    Eigen::MatrixXf control_h;
    Eigen::MatrixXf kalman_gain;
    Eigen::MatrixXf control_P;

    Eigen::MatrixXf control_Q;
    Eigen::MatrixXf control_R;
    Eigen::MatrixXf error_v;
    Eigen::MatrixXf error_w;

    float dtime;
    bool if_first = true;

private:
    void init();

    void init(const float &dtime);

    void get_measure(std::vector<float> &data);
};
}  // namespace fusion

#endif  // DATA_FUSION_FILTER_H
