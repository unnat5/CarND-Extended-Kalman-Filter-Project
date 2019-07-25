#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd z_pred = H_*x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * Ht;
    MatrixXd K = Pht * Si;
    
    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    VectorXd z_pred = VectorXd(3);
  	double px,py,vx,vy;
	px = x_[0],py=x_[1],vx = x_[2], vy = x_[3];
//   cout<<"yo"<<x_[0]<<"ss"<<x_(0)<<endl;
  	double a = sqrt(px*px + py*py);
    double b = atan2(py,px);
  	double c = (px*vx + py*vy)/a;
  	z_pred << a,b,c;
  
  
  
  
    VectorXd y = z - z_pred;
  // y(1) normalization https://discussions.udacity.com/t/rmse-values-of-ekf-project/243997
    while (y(1) > M_PI)
        y(1) -= 2 * M_PI;
    while (y(1) < -M_PI)
        y(1) += 2 * M_PI;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd Pht = P_ * Ht;
    MatrixXd K = Pht * Si;
    
    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_)*P_;
}
