#ifndef KALMAN_H_
#define KALMAN_H_

// include libraries
// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

class KalmanFilter {
public:
  KalmanFilter() {
    // state vector
    x_ = Eigen::VectorXd(4);
    // state covariance matrix
    P_ = Eigen::MatrixXd(4, 4);
    // state transistion matrix
    F_ = Eigen::MatrixXd(4, 4);
    // process covariance matrix
    Q_ = Eigen::MatrixXd(4, 4);
    // measurement matrix
    H_ = Eigen::MatrixXd(2, 4);
    // measurement covariance matrix
    R_ = Eigen::MatrixXd(2, 2);
  }

  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
	    Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
  }

  void Predict(){
    // predict the state
    x_ = F_ * x_;
    Eigen::MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
  }

  void Update(const Eigen::VectorXd &z){
    // update the state by using Kalman Filter equations
    Eigen::VectorXd z_pred = H_ * x_;
    Eigen::VectorXd y = z - z_pred;
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K =  P_ * Ht * Si;

    // new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  }

  // getter functions
  Eigen::VectorXd GetState() {
    return x_;
  }

  Eigen::MatrixXd GetCovariance() {
    return P_;
  }

  private:
    // define variables
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
};

#endif /* KALMAN_H_ */
