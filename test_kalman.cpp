// include libraries
#include <fstream>
#include <stdlib.h>
#include <random>
#include <iostream>
#include <vector>
#include <chrono>
// kalman filter
#include "kalman.hpp"

Eigen::VectorXd GenerateMeasurementData(double delta_t, double x, double y, double vx, double vy, double noise_std) {
  // create a measurement vector
  Eigen::VectorXd z = Eigen::VectorXd(2);
  
  // calculate the new position
  x = x + vx * delta_t;
  y = y + vy * delta_t;

  // add some noise
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count(); // get current time as seed
  std::default_random_engine generator(seed);
  std::normal_distribution<double> distribution(0.0, noise_std);
  double noise_x = distribution(generator);
  double noise_y = distribution(generator);

  std::cout << "noise_x = " << noise_x << std::endl;
  std::cout << "noise_y = " << noise_y << std::endl;

  // assign the new position with noise
  z << x + noise_x, y + noise_y;

  // return the measurement
  return z;
}

// test the code
int main() {
  // create a Kalman Filter instance
  KalmanFilter kf;

  // create a 4D state vector, we don't know yet the values of the x state
  Eigen::VectorXd x = Eigen::VectorXd(4);
  x << 1, 1, 1, 1;

  // state covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  // the initial transition matrix F_
  Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // measurement matrix
  Eigen::MatrixXd H = Eigen::MatrixXd(2, 4);
  H << 1, 0, 0, 0,
       0, 1, 0, 0;

  // measurement covariance
  Eigen::MatrixXd R = Eigen::MatrixXd(2, 2);
  R << 1.1225, 0,
       0, 1.1225;

  // the initial transition matrix Q_
  Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
  Q << 1, 0, 1, 0,
       0, 1, 0, 1,
       1, 0, 1, 0,
       0, 1, 0, 1;

  // call the Kalman Filter init function
  kf.Init(x, P, F, H, R, Q);

  // call the predict function
  kf.Predict();

  // print the output
  std::cout << "x_ = " << std::endl << kf.GetState() << std::endl;
  std::cout << "P_ = " << std::endl << kf.GetCovariance() << std::endl;

  // create measurment vector
  Eigen::VectorXd z = Eigen::VectorXd(2);
  z << 1.2, 0.7;

  // call the update function
  kf.Update(z);

  // print the output
  std::cout << "x_ = " << std::endl << kf.GetState() << std::endl;
  std::cout << "P_ = " << std::endl << kf.GetCovariance() << std::endl;

  // generate some measurements
  std::vector<Eigen::VectorXd> measurements;
  for (size_t i = 0; i < 60; ++i) {
    double index = i * 0.5;
    // generate a measurement
    Eigen::VectorXd z = GenerateMeasurementData(0.1, 0.5*std::pow(index, 2), 0.5*index, 0.1*index, 0.1*index, 1.16);
    // add the measurement to the list
    measurements.push_back(z);
  }

  // plot the results
  std::vector<float> x_est;
  std::vector<float> y_est;
  std::vector<float> x_meas;
  std::vector<float> y_meas;
  
  // kalman filter loop
  std::cout << "Kalman Filter Loop" << std::endl;
  size_t N = measurements.size();
  for (size_t k = 0; k < N; ++k) {
    // call the predict function
    kf.Predict();

    // print the output
    std::cout << "x_ = " << std::endl << kf.GetState() << std::endl;
    std::cout << "P_ = " << std::endl << kf.GetCovariance() << std::endl;

    // call the update function
    kf.Update(measurements[k]);

    // print the output
    std::cout << "x_ = " << std::endl << kf.GetState() << std::endl;
    std::cout << "P_ = " << std::endl << kf.GetCovariance() << std::endl;
    
    // save the data
    x_est.push_back(kf.GetState()(0));
    y_est.push_back(kf.GetState()(1));
    x_meas.push_back(measurements[k](0));
    y_meas.push_back(measurements[k](1));
  }

  // save the data to a file for plotting
  std::ofstream out("data.txt");
  for (size_t i = 0; i < N; ++i) {
    out << x_est[i] << " " << y_est[i] << " " << x_meas[i] << " " << y_meas[i] << std::endl;
  }
  out.close();
  std::cout << "Done!" << std::endl;

  return 0;
}

