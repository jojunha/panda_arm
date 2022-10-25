#pragma once
#ifndef __MODEL_H
#define __MODEL_H

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "custommath.h"


using namespace std;
using namespace Eigen;

class CModel
{
public:
	CModel();
	virtual ~CModel();

    RigidBodyDynamics::Model _model;
    //std::shared_ptr<RigidBodyDynamics::Model> model_;

    void update_kinematics(VectorXd & q, VectorXd & qdot); // update robot state
    void update_dynamics(); // calculate _A, _g, _b, _bg
    void calculate_EE_Jacobians(); // calcule jacobian
    void calculate_EE_positions_orientations(); // calculte End-effector postion, orientation
    void calculate_EE_velocity(); // calculate End-effector velocity

    MatrixXd _A; // inertia matrix
    VectorXd _g; // gravity force vector
	VectorXd _b; // Coriolis/centrifugal force vector
	VectorXd _bg; // Coriolis/centrifugal force vector + gravity force vector

    MatrixXd _J_hand; // jacobian Matrix 6x7
    MatrixXd _J_tmp; 

    Vector3d _position_local_task_hand; // End-effector coordinate
    Vector3d _x_hand; // End-effector position
    Matrix3d _R_hand; // End-effector rotation matrix

    VectorXd _xdot_hand;

    VectorXd _max_joint_torque, _min_joint_torque, _max_joint_velocity, _min_joint_velocity, _max_joint_position, _min_joint_position;

private:
	void Initialize();
	void load_model(); // read URDF model
    void load_model(const Eigen::Ref<const Eigen::Matrix<double, 7, 4> > & dh); // read custom model
    //variables for custom model    
    RigidBodyDynamics::Math::Vector3d com_position_[7];
    RigidBodyDynamics::Math::Vector3d ee_position_;
    Eigen::Vector3d joint_posision_[7];
    Eigen::Matrix3d rot_ee_;   
    unsigned int body_id_[7];
    RigidBodyDynamics::Body body_[7];
    RigidBodyDynamics::Joint joint_[7];
    Eigen::Isometry3d transformDH(const double a, const double d, const double alpha, const double theta);

	void set_robot_config();

    VectorXd _q, _qdot; // joint sensordata
    VectorXd _zero_vec_joint; // zero joint vector

    int _k; // joint number
    int _id_hand; // hand id

    bool _bool_model_update, _bool_kinematics_update, _bool_dynamics_update, _bool_Jacobian_update; // update check

};

#endif