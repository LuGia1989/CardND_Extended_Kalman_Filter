#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

double pi = 3.1415926535897;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  // Forcing y to stay between -pi and +pi
  while (y(1) < -pi) {
    y(1) += 2 * pi;
  }
  while (y(1) > pi) {
    y(1) -= 2 * pi;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double range = sqrt( x_[0]*x_[0] + x_[1]*x_[1] );
  double bearing;
  double range_rate;
  if (fabs(range) > 0.001) {
    bearing = atan2(x_[1] , x_[0]);
    range_rate = ((x_[0] * x_[2] + x_[1] * x_[3]) / range);
  }
  else{
    bearing = 0;
    range_rate = 0;
  }
    MatrixXd z_pred(3, 1);
    z_pred << range, bearing, range_rate;
    VectorXd y = z - z_pred;
     // Forcing y to stay between -pi and +pi
    while (y(1) < -pi) {
      y(1) += 2 * pi;
    }
    while (y(1) > pi) {
      y(1) -= 2 * pi;
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  }
