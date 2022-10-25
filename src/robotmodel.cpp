#include "robotmodel.h"
#define JDOF 7

CModel::CModel()
{
	Initialize();
}

CModel::~CModel()
{
}

void CModel::Initialize()
{
	_bool_model_update = false;
	_bool_kinematics_update = false;
	_bool_dynamics_update = false;
	_bool_Jacobian_update = false;

    _k = JDOF;
    _id_hand = 7;

	_max_joint_torque.setZero(_k);
	_min_joint_torque.setZero(_k);
	_max_joint_velocity.setZero(_k);
	_min_joint_velocity.setZero(_k);
	_max_joint_position.setZero(_k);
	_min_joint_position.setZero(_k);

    _q.setZero(_k);
    _qdot.setZero(_k);
    _zero_vec_joint.setZero(_k);

    _A.setZero(_k,_k);
    _g.setZero(_k);
	_b.setZero(_k);
	_bg.setZero(_k);

    _J_hand.setZero(6,_k);
    _J_tmp.setZero(6,_k);

    _position_local_task_hand.setZero(); // 3x1
    _x_hand.setZero(); // 3x1
    _R_hand.setZero(); // 3x3

    _xdot_hand.setZero(6);

	set_robot_config();

	Eigen::Matrix<double, 7, 4> dh;
  	dh.setZero();
    
	load_model(dh);
}


void CModel::load_model()
{   
  
    RigidBodyDynamics::Addons::URDFReadFromFile("/home/prime/ws_test/src/panda_arm/model/panda.urdf", &_model, false, true);

    cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

    _bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;

}

void CModel::load_model(const Eigen::Ref<const Eigen::Matrix<double, 7, 4> > & dh)
{   
  _model.gravity = Eigen::Vector3d(0., 0., -9.81);

  int kDof = _k;
  double mass[kDof];
  mass[0] = 1.0;
  mass[1] = 1.0;
  mass[2] = 1.0;
  mass[3] = 1.0;
  mass[4] = 1.0;
  mass[5] = 1.0;
  mass[6] = 1.0;

  Eigen::Vector3d global_joint_position[kDof], global_joint_position_new[kDof], axis[kDof];
  Eigen::VectorXd dh_al(7), dh_a(7), dh_d(7), dh_q(7);
  Eigen::Isometry3d transform_joint;
  transform_joint.setIdentity();
  const Eigen::Ref<const Eigen::VectorXd> &a_offset = dh.col(0);
  const Eigen::Ref<const Eigen::VectorXd> &d_offset = dh.col(1);
  const Eigen::Ref<const Eigen::VectorXd> &q_offset = dh.col(2);
  const Eigen::Ref<const Eigen::VectorXd> &alpha_offset = dh.col(3);

  dh_al << 0.0, -1.0 * M_PI_2, M_PI_2, M_PI_2, -1.0 * M_PI_2, M_PI_2, M_PI_2;
  dh_a << 0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088;
  dh_d << 0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0;

  global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
  global_joint_position[1] = Eigen::Vector3d(0.0, 0.0, 0.3330);
  global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
  global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
  global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
  global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

  com_position_[0] = Eigen::Vector3d(0.000096, -0.0346, 0.2575);
  com_position_[1] = Eigen::Vector3d(0.0002, 0.0344, 0.4094);
  com_position_[2] = Eigen::Vector3d(0.0334, 0.0266, 0.6076);
  com_position_[3] = Eigen::Vector3d(0.0331, -0.0266, 0.6914);
  com_position_[4] = Eigen::Vector3d(0.0013, 0.0423, 0.9243);
  com_position_[5] = Eigen::Vector3d(0.0421, -0.0103, 1.0482);
  com_position_[6] = Eigen::Vector3d(0.1, -0.0120, 0.9536);

  for (int i = 0; i < kDof; i++)
  {
    transform_joint = transform_joint * transformDH(dh_a(i) + a_offset(i), dh_d(i) + d_offset(i), dh_al(i) + alpha_offset(i), q_offset(i));
    axis[i] = transform_joint.matrix().block<3, 1>(0, 2);
    global_joint_position_new[i] = transform_joint.translation();
  }

  rot_ee_ = transform_joint.linear();
  ee_position_ = Eigen::Vector3d(0.0, 0.0, 0.107);
  ee_position_ = rot_ee_ * ee_position_;

  joint_posision_[0] = global_joint_position_new[0];
  for (int i = 1; i < kDof; i++)
    joint_posision_[i] = global_joint_position_new[i] - global_joint_position_new[i - 1];

  for (int i = 0; i < kDof; i++)
    com_position_[i] -= global_joint_position[i];

  RigidBodyDynamics::Math::Vector3d inertia[kDof];
  for (int i = 0; i < kDof; i++)
    inertia[i] = Eigen::Vector3d::Identity() * 0.001;

  for (int i = 0; i < kDof; i++)
  {
    body_[i] = RigidBodyDynamics::Body(mass[i], com_position_[i], inertia[i]);
    joint_[i] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, axis[i]);
    if (i == 0)
      body_id_[i] = _model.AddBody(0, RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    else
      body_id_[i] = _model.AddBody(body_id_[i - 1], RigidBodyDynamics::Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
  }

  //RigidBodyDynamics::Addons::URDFReadFromFile("/home/prime/ws_test/src/panda_arm/model/panda.urdf", &_model, false, true);

    cout << endl << endl << "Model Loaded for RBDL." << endl << "Total DoFs: " << _model.dof_count << endl << endl;
	if (_model.dof_count != _k)
	{
		cout << "Simulation model and RBDL model mismatch!!!" << endl << endl;
	}

    _bool_model_update = true; //check model update

	cout << "Model Loading Complete." << endl << endl;

}

Eigen::Isometry3d CModel::transformDH(const double a, const double d, const double alpha, const double theta)
{
  Eigen::Isometry3d transform_dh;
  double st = sin(theta), ct = cos(theta);
  double sa = sin(alpha), ca = cos(alpha);
  transform_dh.setIdentity();
  transform_dh.linear() << ct, -1 * st, 0.0,
      st * ca, ct * ca, -1 * sa,
      st * sa, ct * sa, ca;
  transform_dh.translation() << a, -1 * sa * d, ca * d;
  return transform_dh;
}

void CModel::update_kinematics(VectorXd & q, VectorXd & qdot)
{
	_q = q;
	_qdot = qdot;

	if (_bool_model_update == true)
	{
		RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL); // update kinematics
	}
	else
	{
		cout << "Robot model is not ready. Please load model first." << endl << endl;
	}
	_bool_kinematics_update = true; // check kinematics update
}

void CModel::update_dynamics()
{
	if (_bool_kinematics_update == true)
	{
		RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false); // pdate dynamics
		RigidBodyDynamics::InverseDynamics(_model, _q, _zero_vec_joint, _zero_vec_joint, _g, NULL); // get _g
		RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero_vec_joint, _bg, NULL); // get _g+_b
		_b = _bg - _g; //get _b
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
	_bool_dynamics_update = true; // check kinematics update
}

void CModel::calculate_EE_Jacobians()
{
	if (_bool_kinematics_update == true)
	{
		_J_hand.setZero();
		_J_tmp.setZero();	

		RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _id_hand, _position_local_task_hand, _J_tmp, false); // update kinematc : false
		_J_hand.block<3, 7>(0, 0) = _J_tmp.block<3, 7>(3, 0); // linear : last three entries -> first three entries
		_J_hand.block<3, 7>(3, 0) = _J_tmp.block<3, 7>(0, 0); // angular : first three entries -> last three entries

		_bool_Jacobian_update = true;
	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}

}

void CModel::calculate_EE_positions_orientations()
{
    if (_bool_kinematics_update == true)
	{
		_x_hand.setZero();
		_R_hand.setZero();

		_x_hand = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _id_hand, _position_local_task_hand, false);
		_R_hand = RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _id_hand, false).transpose();

	}
	else
	{
		cout << "Robot kinematics is not ready. Please update kinematics first." << endl << endl;
	}
}

void CModel::calculate_EE_velocity()
{
	if (_bool_Jacobian_update == true)
	{
		_xdot_hand = _J_hand * _qdot;
	}
	else
	{
		cout << "Jacobian matrices are not ready. Please calculate Jacobians first." << endl << endl;
	}
}

void CModel::set_robot_config(){
	_max_joint_position(0) = 166.0 * DEG2RAD;
	_min_joint_position(0) = -166.0 * DEG2RAD;
	_max_joint_position(1) = 101.0 * DEG2RAD;
	_min_joint_position(1) = -101.0 * DEG2RAD;
	_max_joint_position(2) = 166.0 * DEG2RAD;
	_min_joint_position(2) = -166.0 * DEG2RAD;
	_max_joint_position(3) = 176.0 * DEG2RAD;
	_min_joint_position(3) = -4.0 * DEG2RAD;
	_max_joint_position(4) = 166.0 * DEG2RAD;
	_min_joint_position(4) = -166.0 * DEG2RAD;
	_max_joint_position(5) = 215.0 * DEG2RAD;
	_min_joint_position(5) = -1.0 * DEG2RAD;
	_max_joint_position(6) = 166.0 * DEG2RAD;
	_min_joint_position(6) = -166.0 * DEG2RAD;

	_max_joint_velocity(0) = 155.0 * DEG2RAD;
	_max_joint_velocity(1) = 150.0 * DEG2RAD;
	_max_joint_velocity(2) = 150.0 * DEG2RAD;
	_max_joint_velocity(3) = 150.0 * DEG2RAD;
	_max_joint_velocity(4) = 180.0 * DEG2RAD;
	_max_joint_velocity(5) = 180.0 * DEG2RAD;
	_max_joint_velocity(6) = 180.0 * DEG2RAD;
	_min_joint_velocity = -_max_joint_velocity;

}
