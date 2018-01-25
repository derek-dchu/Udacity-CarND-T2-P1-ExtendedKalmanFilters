#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

void KalmanFilter::UpdateWithY(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
  UpdateWithY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double ro = sqrt(pow(px,2)+pow(py,2));
  double theta = atan2f(py,px);
  double ro_dot = (px*vx + py*vy) / ro;
  VectorXd x_dot = VectorXd(3);
  x_dot << ro, theta, ro_dot;
  VectorXd y = z - x_dot;

  // make sure the theta of y is within [-pi, pi]
  while (y(1) > M_PI) {
    y(1) -= M_PI;
  }
  while (y(1) < -M_PI) {
    y(1) += M_PI;
  }

  UpdateWithY(y);
}

void KalmanFilter::UpdateQ(double dt) {
  double dt_2 = dt * dt; //dt^2
  double dt_3 = dt_2 * dt; //dt^3
  double dt_4 = dt_3 * dt; //dt^4
  double dt_4_4 = dt_4 / 4; //dt^4/4
  double dt_3_2 = dt_3 / 2; //dt^3/2

  //Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  //set the process covariance matrix Q
  Q_ = MatrixXd(4, 4);
  Q_ <<  dt_4_4*noise_ax, 0, dt_3_2*noise_ax, 0,
         0, dt_4_4*noise_ay, 0, dt_3_2*noise_ay,
         dt_3_2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3_2*noise_ay, 0, dt_2*noise_ay;
}