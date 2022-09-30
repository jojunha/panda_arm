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
  // robot.setCollisionBehavior(
  //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
  //     {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
  //     {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  double current_time = 0.0;
  bool is_first = true;

  Eigen::VectorXd q_goal, qdot_goal;
  q_goal.setZero(7);
  qdot_goal.setZero(7);

  q_goal(0) = 0.0;
  q_goal(1) = 0.0;
  q_goal(2) = 0.0;
  q_goal(3) = -PI/2.;
  q_goal(4) = 0.0;
  q_goal(5) = PI/2.;
  q_goal(6) = 0.0;

  // define callback for the position control loop
  std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
      position_control_callback = [&](const franka::RobotState& robot_state,
                                        franka::Duration duration) -> franka::JointPositions {

    Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

    cout << controller.getDesiredPosition() << endl;
    cout << controller.getDesiredPosition() << endl;
    if(is_first)
    {
      controller.setGoalJoint(q_goal, qdot_goal, q, dq);
      controller.compute();
      std::cout << "init position" << std::endl <<
      q.transpose() << std:: endl;
      is_first = false;
      return franka::JointPositions(robot_state.q);
    }

    control_mode_mutex.lock();


    current_time += duration.toSec();

    cout << "current_time : " << current_time << endl;

    controller.read(current_time,q,dq);
    controller.compute();

    std::array<double, 7> position_d_array{};
    Eigen::Matrix<double, 7, 1>::Map(position_d_array.data()) = controller.getDesiredPosition();

    control_mode_mutex.unlock();


    franka::JointPositions output(position_d_array);

    cout << controller.isFinished() << endl;
    if(controller.isFinished())
    {
      cout << "motion finish!!" << endl;
      return franka::MotionFinished(output);
    }

    return output;
  };
  try{
    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    cout << "control start!!" << endl;
    // TODO: position mode : 
    robot.control(position_control_callback);
  } 
  catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}  