#include <iostream>
#include <string>
#include <memory>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include "position_controller.h"


using namespace std;

bool running = true;

CController controller;
std::mutex control_mode_mutex;

int main()
{
  cout << "Set Panda!" << endl;
  franka::Robot robot("172.16.0.2");
  cout << "Connect Panda!" << endl;
  cout << "start!" << endl;

  franka::Model model = robot.loadModel();

  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  double current_time = 0.0;
  bool is_first = true;

  int control_mode = 1;

  Eigen::VectorXd q_goal, qdot_goal;
  q_goal.setZero(7);
  qdot_goal.setZero(7);
  q_goal(0) = 0;
  q_goal(1) = 0;
  q_goal(2) = 0;
  q_goal(3) = - PI/2.;
  q_goal(4) = 0;
  q_goal(5) = + PI/2.;
  q_goal(6) = 0;

  Eigen::VectorXd x_goal, xdot_goal;
  x_goal.setZero(6);
  xdot_goal.setZero(6);

  // define callback for the position control loop
  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      position_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration duration) -> franka::JointPositions {

    Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

    if(is_first)
    {
      if(control_mode == 1)
      {
        controller.setGoalJoint(q_goal, qdot_goal);
      }
      else if(control_mode == 2)
      {
        controller.setGoalEE(x_goal, xdot_goal);
      }

      controller.read(current_time,q,dq);
      controller.compute();

      is_first = false;
      return franka::JointPositions(robot_state.q);
    }

    control_mode_mutex.lock();


    current_time += duration.toSec();

    controller.read(current_time,q,dq);
    controller.compute();

    Eigen::Map<const Eigen::Matrix<double, 4, 4> > ht(robot_state.O_T_EE.data());

    std::array<double, 7> position_d_array{};
    Eigen::Matrix<double, 7, 1>::Map(position_d_array.data()) = controller.getDesiredPosition();

    control_mode_mutex.unlock();


    franka::JointPositions output(position_d_array);

    if(controller.isFinished())
    {
      std::cout << "finished EE pose :" << std::endl << controller.getEEPosition().transpose() << std:: endl;
      std::cout << std::endl;
      return franka::MotionFinished(output);
    }

    return output;
  };
  
  try{
    // start real-time control loop
    // std::cout << "WARNING: Collision thresholds are set to high values. "
    //           << "Make sure you have the user stop at hand!" << std::endl
    //           << "After starting try to push the robot and see how it reacts." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    
    // std::cin.ignore();

    // joint motion
    cout << "joint controll start!!" << endl;
    robot.control(position_control_callback);

    // // EE motion
    // is_first = true;
    // control_mode = 1;
    // q_goal(0) = 5.0*DEG2RAD;
    // q_goal(1) = 5.0*DEG2RAD;
    // q_goal(2) = 5.0*DEG2RAD;
    // q_goal(3) = 10.0*DEG2RAD - PI/2.;
    // q_goal(4) = 5.0*DEG2RAD;
    // q_goal(5) = -10.0*DEG2RAD + PI/2.;
    // q_goal(6) = 5.0*DEG2RAD;
    // qdot_goal.setZero(7);

    // current_time = 0.0;

    // while(1)
    // {}
    // x_goal = controller.getEEPosition();
    // cout << "EE current" << controller.getEEPosition().transpose() << endl;

    is_first = true;
    control_mode = 2;
    x_goal = controller.getEEPosition();
    cout << "current EE pose :"<< std::endl << controller.getEEPosition().transpose() << endl;
    x_goal(0) += 0.15;
    x_goal(2) -= 0.4;
    // x_goal(2) -= 0.15;
    current_time = 0.0;

    // std::cout << "WARNING: Collision thresholds are set to high values. "
    //           << "Make sure you have the user stop at hand!" << std::endl
    //           << "After starting try to push the robot and see how it reacts." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();


    // cout << "operational controll start!!" << endl;
    robot.control(position_control_callback);

    is_first = true;
    control_mode = 2;
    x_goal = controller.getEEPosition();
    cout << "current EE pose :"<< std::endl << controller.getEEPosition().transpose() << endl;
    // x_goal(0) += 0.15;
    x_goal(2) -= 0.05;
    // x_goal(2) -= 0.15;
    current_time = 0.0;

    // std::cout << "WARNING: Collision thresholds are set to high values. "
    //           << "Make sure you have the user stop at hand!" << std::endl
    //           << "After starting try to push the robot and see how it reacts." << std::endl
    //           << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();


    // cout << "operational controll start!!" << endl;
    robot.control(position_control_callback);


    // Eigen::VectorXd x_finish;
    // x_finish = controller.getEEPosition();
    // for(int i = 0; i < 7; i++)
    // {
    //   cout << x_goal(i) - x_finish(i) << endl;
    // }

  } 
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}  