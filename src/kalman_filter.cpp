#include "kalman_filter.h"
#include <iostream>

using namespace std;
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
    cout << "x before predict" << x_ << endl;
    x_ = F_ * x_;
    MatrixXd Ft_(4,4);
    Ft_ = F_.transpose();
    P_ = F_ * P_ * Ft_ + Q_;
    cout << "x after predict" << x_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
    MatrixXd Ht_(4,2);
    Ht_ = H_.transpose();
    VectorXd y = z - H_ * x_;
    MatrixXd S_(2,2);
    S_ = H_ * P_ * Ht_ + R_;
    MatrixXd S_i(2,2);
    S_i = S_.inverse();
    MatrixXd K_(4,2);
    K_ = P_ * Ht_ * S_i;
    MatrixXd I_(4, 4);
    I_.setIdentity(4,4);
    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    if (x_(0)==0) { x_(0) = 0.00001;}
    if (x_(1)==0) { x_(1) = 0.00001;}
    if (x_(2)==0) { x_(2) = 0.00001;}
    VectorXd h_x(3);
    h_x << sqrt((pow(x_(0), 2.0) + pow(x_(1), 2.0))),
            atan2(x_(1), x_(0)),
            (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt((pow(x_(0), 2.0) + pow(x_(1), 2.0)));

    MatrixXd Ht_(4,3);
    Ht_ = H_.transpose();
    VectorXd y(3);
    y = z - h_x;
    cout << "measured: " << z << endl;
    cout << "predicted: " << h_x << endl;
    MatrixXd S_(3,3);
    S_ = H_ * P_ * Ht_ + R_;

    MatrixXd S_i(3,3);
    S_i = S_.inverse();

    MatrixXd K_(4,3);
    K_ = P_ * Ht_ * S_i;

    MatrixXd I_(4, 4);
    I_.setIdentity(4,4);

    x_ = x_ + K_ * y;
    P_ = (I_ - K_ * H_) * P_;
    cout << "optim: " << x_ << endl;
}
//0.999586    0.0287562            0            0
//-0.00352309     0.122465            0            0
//9.58419e-20 -3.33153e-18     0.999586    0.0287562
