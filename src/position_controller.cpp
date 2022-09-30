#include "position_controller.h"
#include <chrono>

#include <fstream> // ifstream header
#include <iostream>
#include <string> // getline header

CController::CController()
{
	_k = 7;
	Initialize();
}

CController::~CController()
{
}

void CController::setGoalJoint(VectorXd q_goal, VectorXd qdot_goal, VectorXd q, VectorXd qdot)
{
    _q_goal = q_goal;
    _qdot_goal = qdot_goal;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q(i);
		_qdot(i) = qdot(i);		
	}

    _bool_init = true;
    _bool_finish = false;

    _control_mode = 1;
}

void CController::setGoalEE(VectorXd x_goal_hand, VectorXd xdot_goal_hand, VectorXd q, VectorXd qdot)
{
    _x_goal_hand = x_goal_hand;
    _xdot_goal_hand = xdot_goal_hand;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q(i);
		_qdot(i) = qdot(i);		
	}
	ModelUpdate();
	
    _bool_init = true;
    _bool_finish = false;

    _control_mode = 2;
}

bool CController::isFinished()
{
    return _bool_finish;
}

void CController::read(double t, VectorXd q, VectorXd qdot)
{	
	_t = t;
	if (_bool_init == true)
	{
		_init_t = _t;
	}

	_dt = t - _pre_t;
	_pre_t = t;

	for (int i = 0; i < _k; i++)
	{
		_q(i) = q(i);
		_qdot(i) = qdot(i);		
	}
}

VectorXd CController::getDesiredPosition()
{
	return _q_des;
}


void CController::compute()
{
    ModelUpdate();

	if(_control_mode == 1) //joint space control
	{
		if (_t - _init_t < 0.1 && _bool_init == true) 
		{
			cout << "motion first" << endl;
			_start_time = _init_t;
            _motion_time = 5.0;
			_end_time = _start_time + _motion_time;
			JointTrajectory.reset_initial(_start_time, _q, _qdot);
			JointTrajectory.update_goal(_q_goal, _qdot_goal, _end_time);
			_bool_init = false;
		}
		JointTrajectory.update_time(_t);
		_q_des = JointTrajectory.position_cubicSpline();
		_qdot_des = JointTrajectory.velocity_cubicSpline();

		if (JointTrajectory.check_trajectory_complete() == 1)
		{
			_bool_finish = true;
		}
	}
	else if(_control_mode == 2)
	{
		if (_t - _init_t < 0.1 && _bool_init == true)
		{
			_start_time = _init_t;
            _motion_time = 5.0;
			_end_time = _start_time + _motion_time;
			HandTrajectory.reset_initial(_start_time, _x_hand, _xdot_hand);
			HandTrajectory.update_goal(_x_goal_hand, _xdot_goal_hand, _end_time);
			_bool_init  = true;
		}

		HandTrajectory.update_time(_t);
		_x_des_hand = HandTrajectory.position_cubicSpline();
		_xdot_des_hand = HandTrajectory.velocity_cubicSpline();

		if (HandTrajectory.check_trajectory_complete() == 1)
		{
            _bool_finish = true;
		}

		CLIK();
	}
}

void CController::ModelUpdate()
{
    Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
    Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	_J_hands = Model._J_hand;

	_x_hand.head(3) = Model._x_hand;
	_x_hand.tail(3) = CustomMath::GetBodyRotationAngle(Model._R_hand);

	_xdot_hand = Model._xdot_hand;

}


void CController::CLIK()
{
	_J_bar_hands = CustomMath::pseudoInverseQR(_J_hands);

	_qdot_des = _J_bar_hands*(_xdot_des_hand + _x_kp*(_x_des_hand - _x_hand));
	_q_des = _q + _dt*_qdot_des;
}

void CController::Initialize()
{
    _control_mode = 1; //1: joint space, 2: operational space

	_bool_init = false;
	_t = 0.0;
	_init_t = 0.0;
	_pre_t = 0.0;
	_dt = 0.0;

	_x_kp = 20.0;

    _q.setZero(_k);
	_qdot.setZero(_k);

	_J_hands.setZero(6,_k);
	_J_bar_hands.setZero(_k,6);

	_x_hand.setZero(6);
	_xdot_hand.setZero(6);


	_start_time = 0.0;
	_end_time = 0.0;
	_motion_time = 0.0;

	_q_des.setZero(_k);
	_qdot_des.setZero(_k);
	_q_goal.setZero(_k);
	_qdot_goal.setZero(_k);

	_x_des_hand.setZero(6);
	_xdot_des_hand.setZero(6);
	_x_goal_hand.setZero(6);
	_xdot_goal_hand.setZero(6);

	JointTrajectory.set_size(_k);
	HandTrajectory.set_size(6);


    _bool_finish = false;
}
