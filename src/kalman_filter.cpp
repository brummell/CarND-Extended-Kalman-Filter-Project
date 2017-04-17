#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    auto H_t = H_.transpose();
    auto y = z - H_ * x_;
    auto S_ = H_ * P_ * H_t + R_;
    auto S_i = S_.transpose();
    auto K_ = P_ * H_t * S_i;
    MatrixXd::Identity I_(4, 4);
    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    auto H_t = H_.transpose();
    auto y = z - H_ * x_;
    auto S_ = H_ * P_ * H_t + R_;
    auto S_i = S_.transpose();
    auto K_ = P_ * H_t * S_i;
    MatrixXd::Identity I_(4, 4);
    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_) * P_;
}
