/*!
 * \file attitude_ekf.h
 *
 * Vicon filter attitude EKF
 *
 * Created on: Oct 14, 2013
 *     Author: Mark Cutler
 *      Email: markjcutler@gmail.com
 */

#ifndef ATTITUDE_EKF_H_
#define ATTITUDE_EKF_H_

#include <boost/bind.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <valarray>

#include "mocap/defines.h"
#include "mocap/utils.h"

/**
 *  \brief EKF on vicon attitude
 *
 *  Implements a standard EKF to filter raw vicon outputs.
 *
 *  State: [p, q, r, qx, qy, qz, q0]
 *  Measurement: [qx, qy, qz, q0]
 *
 *  Attitude propagation equations based on this paper:
 *  "AN EXTENDED KALMAN FILTER FOR QUATERNION-BASED ATTITUDE ESTIMATION"
 *  by Joao L. Marins
 */
class AttitudeEKF {
public:
  AttitudeEKF();
  void measUpdate(geometry_msgs::Quaternion q_meas);
  // void measUpdate(geometry_msgs::Quaternion q_meas, bool occ);
  void propUpdate(double dt);
  geometry_msgs::Quaternion getAtt();
  geometry_msgs::Vector3 getRate();
  void reset();

private:
  bool initialized_;
  double actual_dt;
  Eigen::Matrix<double, ATT_STATES, 1>
  processModel(Eigen::Matrix<double, ATT_STATES, 1> x);
  Eigen::Matrix<double, ATT_STATES, ATT_STATES>
  transitionMatrix(Eigen::Matrix<double, ATT_STATES, 1> x);
  std::valarray<double> xQdot(double dt, std::valarray<double> x);
  std::valarray<double> vec(Eigen::MatrixXd m);
  Eigen::MatrixXd unvec(std::valarray<double> v);
  unsigned int largestRoot(unsigned int lenV);

  // int flip; ///<   Should the current measured quaternion be flipped or not
  Eigen::Quaterniond q_prev; ///< Previously measured quaternion

  Eigen::Matrix<double, ATT_STATES, ATT_STATES>
      Wc; ///< 7x7 process noise matrix
  Eigen::Matrix<double, ATT_MEAS, ATT_MEAS>
      Rd; ///< 4x4 measurement noise matrix

  // Internal variables
  Eigen::Matrix<double, ATT_STATES, 1> xhatm;       ///< xhat minus
  Eigen::Matrix<double, ATT_STATES, 1> xhatp;       ///< xhat plus
  Eigen::Matrix<double, ATT_STATES, ATT_MEAS> L;    ///< Kalman gain
  Eigen::Matrix<double, ATT_STATES, ATT_STATES> Qm; ///< Q minus
  Eigen::Matrix<double, ATT_STATES, ATT_STATES> Qp; ///< Q plus
  Eigen::Matrix<double, ATT_MEAS, ATT_STATES> C;    ///< Measurement matrix
};

#endif /* ATTITUDE_EKF_H_ */
