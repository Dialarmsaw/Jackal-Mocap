/*
 * GHKFilter.h
 * Implements a very simple constant acceleration and constant rate ghk filter.
 * From: "TRACKING AND KALMAN FILTERING MADE EASY" by Eli Brookner -- pages
 * 51-55
 *
 *  Created on: Sep 23, 2013
 *      Author: mark
 */

#ifndef GHKFILTER_H_
#define GHKFILTER_H_

#include "mocap/defines.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

class GHKFilter {
public:
  GHKFilter();
  void propUpdate(double dt);
  void measUpdate(double dt, geometry_msgs::Point measure_pos);
  geometry_msgs::Point getPos() { return pos; }
  geometry_msgs::Vector3 getVel() { return vel; }
  geometry_msgs::Vector3 getAcc() { return acc; }
  void reset();

private:
  bool initialized_;
  geometry_msgs::Point pos;
  geometry_msgs::Vector3 vel;
  geometry_msgs::Vector3 acc;

  void propUpdate_single(double dt, double &xk, double &vk, double &ak);
  void measUpdate_single(double dt, double xm, double &xk, double &vk,
                         double &ak);

  double g_gain, h_gain, k_gain;
};

#endif /* GHKFILTER_H_ */
