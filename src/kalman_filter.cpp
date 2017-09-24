#include <iostream>
#include "kalman_filter.h"

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  // Move Ht out to static computation
  // MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * H_laser_t_ + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * H_laser_t_;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = atan2(py, px);

  VectorXd z_pred(3);
  z_pred << c2, c3, (px * vx + py * vy) / c2;
  VectorXd y = z - z_pred;
  // The error here when phi of is +pi and -pi for z and z_pred should be small.
  // This is the discontinuity of atan at +/- pi
  // Ah, I was only incrementing by M_PI, not 2 * M_PI
  while (y[1] > M_PI) {
    y[1] -= 2 * M_PI;
  }
  while (y[1] < - M_PI) {
    y[1] += 2 * M_PI;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
