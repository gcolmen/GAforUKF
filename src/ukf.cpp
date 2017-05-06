#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  //if the filter is initialized yet or not
  is_initialized_ = false;
  //the time stamp of the last sensor read
  previous_timestamp_ = 0;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2; //optimal found so far: 0.544; // 2;// 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.55; //optimal found so far: 0.519104; // 0.55;// 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;//0.15

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //The state dimension
  n_x_ = 5;
  //The augmented dimension
  n_aug_ = 7;
  //The predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  //Initialize P, the covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  /*P_ << 1, 0, 0 ,0 ,0,
	    0, 1, 0, 0, 0,
	    0, 0, 1, 0, 0,
	    0, 0, 0, 1, 0,
	    0, 0, 0, 0, 1;*/
  P_ << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020,
	  -0.0013, 0.0077, 0.0011, 0.0071, 0.0060,
	  0.0030, 0.0011, 0.0054, 0.0007, 0.0008,
	  -0.0022, 0.0071, 0.0007, 0.0098, 0.0100,
	  -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;
  /* Optimal values found so far:
  P_ << 0.00181339, - 0.00182706,    0.0297646, - 0.0010674, - 0.000900672,
	  - 0.00182706,    0.0145208,   0.00109611,   0.00407719,    0.0106453,
	  0.0297646,   0.00109611,   0.00190225,   0.00111579,   0.00032981,
	  - 0.0010674,   0.00407719,   0.00111579,   0.00219884,   0.00300394,
	  - 0.000900672,    0.0106453,   0.00032981,   0.00300394 ,   0.0164061;*/

  //The weights
  weights_ = VectorXd(2 * n_aug_ + 1);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	/*If the sensor type is lidar there are two points in the vector
	If it's radar there are three*/
   /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
	

    // first measurement
    cout << "UKF: " << endl;
    //x_ = VectorXd(4);
    x_ << 1, 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float ro = meas_package.raw_measurements_(0);
		float phi = meas_package.raw_measurements_(1);
		float rhodot = meas_package.raw_measurements_(2);
		//Modify for RADAR
		x_ << ro * cos(phi), ro * sin(phi), rhodot * cos(phi), rhodot * sin(phi), 0;
		
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;

    }

	previous_timestamp_ = meas_package.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  //For the rest of cases (not the first measurement)
  float dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;
  //Prediction
  Prediction(dt);

  //Update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
	  UpdateRadar(meas_package);
  }
  else
  {
	  UpdateLidar(meas_package);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	//1- GENERATE sigma points (in Xsig_pred)
	//create augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);

	//create augmented state covariance
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

	//create augmented mean state
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_*std_a_;
	P_aug(6, 6) = std_yawdd_*std_yawdd_;
	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
	}

	//2- Predict sigma points
	//predict sigma points
	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}

	//3- Predict the state, and the state covariance matrix.
	//create vector for predicted state
	VectorXd x = VectorXd(n_x_);
	//create covariance matrix for prediction
	MatrixXd P = MatrixXd(n_x_, n_x_);
	// set weights
	double weight_0 = lambda_ / (lambda_ + n_aug_);
	weights_(0) = weight_0;
	for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
		double weight = 0.5 / (n_aug_ + lambda_);
		weights_(i) = weight;
	}

	//predicted state mean
	x.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
		x = x + weights_(i) * Xsig_pred_.col(i);
	}

	//predicted state covariance matrix
	P.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

											   // state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		P = P + weights_(i) * x_diff * x_diff.transpose();
	}
	//4- Update the state and the covariance matrix
	x_ = x;
	P_ = P;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  MatrixXd R_laser_ = MatrixXd(2, 2);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
	  0, 0.0225;
  //measurement matrix
  MatrixXd H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
		0, 1, 0, 0, 0;
	
  //Lidar reading
  int n_z = 2;
  VectorXd z = VectorXd(n_z);
  float px = meas_package.raw_measurements_(0);
  float py = meas_package.raw_measurements_(1);
  //float yaw = meas_package.raw_measurements_(2);
  //Modify for LIDAR
  z << px, py; // , yaw;

	VectorXd z_pred = H_laser_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_laser_.transpose();
	MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_laser_) * P_;
	
	//Update NIS
	VectorXd z_diff = z - z_pred;
	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											 // extract values for better readibility
	  double p_x = Xsig_pred_(0, i);
	  double p_y = Xsig_pred_(1, i);
	  double v = Xsig_pred_(2, i);
	  double yaw = Xsig_pred_(3, i);

	  double v1 = cos(yaw)*v;
	  double v2 = sin(yaw)*v;

	  // measurement model
	  Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
	  Zsig(1, i) = atan2(p_y, p_x);                                 //phi
	  Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
	  z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
											 //residual
	  VectorXd z_diff = Zsig.col(i) - z_pred;

	  //angle normalization
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	  S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_, 0, 0,
	  0, std_radphi_*std_radphi_, 0,
	  0, 0, std_radrd_*std_radrd_;
  S = S + R;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											 //residual 
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  //angle normalization
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	  // state difference
	  VectorXd x_diff = Xsig_pred_.col(i) - x_;
	  //angle normalization
	  while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

	  Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //Radar reading
  VectorXd z = VectorXd(n_z);
  float ro = meas_package.raw_measurements_(0);
  float phi = meas_package.raw_measurements_(1);
  float rhodot = meas_package.raw_measurements_(2);
  //Modify for RADAR
  z << ro, phi, rhodot;

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //Update NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}
