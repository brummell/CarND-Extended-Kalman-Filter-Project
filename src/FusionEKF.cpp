#include "FusionEKF.h"
#include "tools.h"
//#include "cmath"
#include "Eigen/Dense"
#include <iostream>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "IncompatibleTypes"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    Hj_ << 0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0;
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    ekf_.F_ = MatrixXd(4,4);
    ekf_.F_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    noise_ax = 9;
    noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        ekf_.P_= MatrixXd(4,4);
        ekf_.P_ << 10000, 0, 0, 0,
                0, 10000, 0, 0,
                0, 0, 10000, 0,
                0, 0, 0, 10000;

        ekf_.Q_= MatrixXd(4,4);

        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            auto rho = measurement_pack.raw_measurements_(0);
            auto phi = measurement_pack.raw_measurements_(1);
            auto rho_dot = measurement_pack.raw_measurements_(2);

            ekf_.x_ << rho * cos(phi),
                    rho * sin(phi),
                    rho_dot * cos(phi), // seems a slight bit better initialization than 0s at basically no cost.
                    rho_dot * sin(phi);

        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_ << measurement_pack.raw_measurements_(0),
                    measurement_pack.raw_measurements_(1),
                    0,
                    0;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    //compute the time elapsed between the current and previous measurements
    auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the process covariance matrix Q
    ekf_.Q_ << pow(dt, 4.0) / 4 * noise_ax, 0, pow(dt, 3.0) / 2 * noise_ax, 0,
            0, pow(dt, 4.0) / 4 * noise_ay, 0, pow(dt, 3.0) / 2 * noise_ay,
            pow(dt, 3.0) / 2 * noise_ax, 0, pow(dt, 2.0) * noise_ax, 0,
            0, pow(dt, 3.0) / 2 * noise_ay, 0, pow(dt, 2.0) * noise_ay;


    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        auto tools = Tools();
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        cout << "RADAR" << endl;
    } else {
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
        cout << "LIDAR" << endl;


    }
}
#pragma clang diagnostic pop

#pragma clang diagnostic pop