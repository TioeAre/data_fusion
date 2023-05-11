//
// Created by tioeare on 5/9/23.
//
#include "filter.h"

fusion::filter::filter(float dtime2) : dtime(dtime2) { init(); }

void fusion::filter::update(std::vector<float> &data, const float &dtime1, const Eigen::MatrixXf &control_H1,
                            std::vector<double> &new_data) {
    control_H = control_H1;
    init(dtime1);
    get_measure(data);
    if (if_first) {
        if_first = false;
        return;
    }
    hat_x = control_A * hat_x;
    control_P = control_A * control_P * control_A.transpose() + error_w * control_Q * error_w.transpose();
    kalman_gain = control_P * control_h.transpose() *
                  (control_h * control_P * control_h.transpose() + error_v * control_R * error_v.transpose()).inverse();
    hat_x = hat_x + kalman_gain * (measure_x - control_h * hat_x);
    control_P = (Eigen::MatrixXf::Identity(6, 6) - kalman_gain * control_h) * control_P;
    new_data[0] = hat_x(0);
    new_data[1] = hat_x(1);
    new_data[2] = hat_x(2);
}

void fusion::filter::get_measure(std::vector<float> &data) {
    Eigen::VectorXf measure = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(data.data(), (long)data.size());
    measure_x = control_H * measure;
    if (if_first) {
        hat_x = measure_x;
    }
}

void fusion::filter::init(const float &dtime1) {
    control_A << 1, 0, 0, dtime1, 0, 0, 0, 1, 0, 0, dtime1, 0, 0, 0, 1, 0, 0, dtime1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 1;
}

void fusion::filter::init() {
    hat_x.resize(6);
    hat_x.setZero();
    measure_x.resize(6);
    measure_x.setZero();
    control_A.resize(6, 6);
    control_A.setZero();
    control_H.resize(6, 14);
    control_h.resize(6, 6);
    kalman_gain.resize(6, 6);
    kalman_gain.setZero();

    control_P.resize(6, 6);
    control_Q.resize(6, 6);
    control_R.resize(6, 6);
    error_v.resize(6, 6);
    error_w.resize(6, 6);

    control_H << 1. / 3, 0, 0, 1. / 3, 0, 0, 1. / 3, 0, 0, 0, 0, 0, 0, 0, 0, 1. / 3, 0, 0, 1. / 3, 0, 0, 1. / 3, 0, 0,
        0, 0, 0, 0, 0, 0, 1. / 3, 0, 0, 1. / 3, 0, 0, 1. / 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1. / 2, 0, 0,
        1. / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1. / 2, 0, 0, 1. / 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;

    control_A << 1, 0, 0, dtime, 0, 0, 0, 1, 0, 0, dtime, 0, 0, 0, 1, 0, 0, dtime, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    control_h << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1;
    control_P << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1;
    control_Q << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1;
    control_R << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1;
    error_v << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        1;
    error_w << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        1;
}
