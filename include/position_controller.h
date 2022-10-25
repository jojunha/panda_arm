#pragma once
#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "robotmodel.h"
#include "trajectory.h"
#include "custommath.h"

using namespace std;
using namespace Eigen;

#define NECS2SEC 1000000000

class CController
{

public:
    CController();
    virtual ~CController();	

    void setGoalJoint(VectorXd q_goal, VectorXd qdot_goal);
    void setGoalEE(VectorXd x_goal_hand, VectorXd xdot_goal_hand);


    void read(double time, VectorXd q, VectorXd qdot);
    void compute();
    VectorXd getDesiredPosition();
    VectorXd getEEPosition();

    bool isFinished();

private:
    void Initialize();
    void ModelUpdate();

    VectorXd _q; // joint angle
	VectorXd _qdot; // joint velocity
    VectorXd _torque; // joint torque

    int _k; // DOF

    bool _bool_init;
    bool _bool_finish;
    double _t;
    double _dt;
	double _init_t;
	double _pre_t;

    //controller
    double _x_kp; // task control P gain

    void CLIK();

    // robotmodel
    CModel Model;

    int _control_mode; //1: joint space, 2: operational space

    //motion trajectory
	double _start_time, _end_time, _motion_time;

    CTrajectory JointTrajectory; // joint space trajectory
    CTrajectory HandTrajectory; // task space trajectory

    bool _bool_joint_motion, _bool_ee_motion; // motion check

    VectorXd _q_des, _qdot_des; 
    VectorXd _q_goal, _qdot_goal;
    VectorXd _x_des_hand, _xdot_des_hand;
    VectorXd _x_goal_hand, _xdot_goal_hand;

    MatrixXd _J_hands; // jacobian matrix
    MatrixXd _J_bar_hands; // pseudo invere jacobian matrix

    VectorXd _x_hand, _xdot_hand; // End-effector

    VectorXd _x_err_hand;
    Matrix3d _R_des_hand;


};

#endif
