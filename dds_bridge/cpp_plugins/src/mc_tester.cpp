// test_motor_controller.cpp

#include "../include/motor_controller.h" // your header
#include <iostream>
#include <exception>
#include <array>

int main()
{
  try
  {
    // sleep(5);
    // 1) Instantiate
    // MotorController mc({1.7, 1.7});
    MotorController mc;
    std::cout << "MotorController constructed successfully\n";

    // 2) Test single‐motor move (will block until position or throw)
    // std::cout << "→ Calling SetPosition(0.5 rad, accel=1.0, id=0)\n";
    mc.SetMotorPosition(0.0, 1.0, 0);
    mc.SetMotorPosition(0.0, 1.0, 1);
    std::cout << "✔ SetPosition completed\n\n";
    sleep(1);
    mc.SetMotorsStop();
    sleep(0.1);
    // 3) Test full‐arm pose
    // double poses[MotorController::MOTORS] = {
    //     0.1, 0.2, 0.3, 0.4, 0.5, 0.6
    // };
    std::array<double, MotorController::getMOTORS()> poses_pi = {3.14, -3.14};
    std::array<double, MotorController::getMOTORS()> poses_zero = {0.0, 0.0};
    double accel = 5.0;

    mc.SetMotorPositions(poses_pi, accel);
    sleep(3);
    printf(mc.getTrajectoryComplete()? "Trajectory Complete" : "Trajectory incomplete");
    mc.SetMotorPositions(poses_zero, accel);
    // while (true)
    // {
    //   // std::cout << "→ Calling SetArmPose("<<poses_pi[0]", "<<", accel="<<0.5<<")\n";
    //   // printf("Calling SetArmPose(%2.2f, %2.2f, accel=%2.2f)\n", poses_pi[0], poses_pi[1], 0.5);

    //   mc.SetMotorPositions(poses_pi, accel);
    //   // std::cout << "✔ SetArmPose completed\n";

    //   // while (!(((mc.getMotorStatuses()[0].mode == moteus::Mode::kPosition) && mc.getMotorStatuses()[0].trajectory_complete) &&
    //   //          ((mc.getMotorStatuses()[1].mode == moteus::Mode::kPosition) && mc.getMotorStatuses()[1].trajectory_complete)))
    //   // while (true)
    //   // {
    //     for (u_int64_t i = 0; i < 1001; i++)
    //     {
    //       ::printf("M0 %3d p/v/t/traj/polling=(%7.3f,%7.3f,%7.3f,%c,%c) \t M1 %3d p/v/t/traj/polling=(%7.3f,%7.3f,%7.3f,%c,%c)\n",
    //                static_cast<int>(mc.getMotorStatuses()[0].mode),
    //                mc.getMotorStatuses()[0].position,
    //                mc.getMotorStatuses()[0].velocity,
    //                mc.getMotorStatuses()[0].torque,
    //                mc.getMotorStatuses()[0].trajectory_complete ? 't' : 'f',
    //                mc.getPollMotors() ? 't' : 'f',
    //                static_cast<int>(mc.getMotorStatuses()[1].mode),
    //                mc.getMotorStatuses()[1].position,
    //                mc.getMotorStatuses()[1].velocity,
    //                mc.getMotorStatuses()[1].torque,
    //                mc.getMotorStatuses()[1].trajectory_complete ? 't' : 'f',
    //                mc.getPollMotors() ? 't' : 'f');
    //       ::fflush(stdout);
    //       // ::usleep(10);
    //     }
    //     // printf(mc.getPollMotors() ? "true" : "false");
    //     mc.setPollMotors(false);
    //     ::usleep(10);
    //     printf(mc.getPollMotors() ? "true" : "false");
    //     ::usleep(10);
    //   // }
    //   // {
    //   //   break;
    //   // }
    //   // printf("Calling SetArmPose(%2.2f, %2.2f, accel=%2.2f)\n", poses_zero[0], poses_zero[1], 0.5);
    //   // mc.SetMotorPositions(poses_zero, accel);
    //   // sleep(0.1);
    //   // mc.ReleaseArmTorque();
    // }
    sleep(0.1);
    // std::cout << "Releasing Motor Torque\n";
    mc.SetMotorsStop();
  }
  catch (const std::exception &e)
  {
    std::cerr << "EXCEPTION: " << e.what() << "\n";
    return 1;
  }

  return 0;
}

// #include <unistd.h>

// #include <iostream>

// #include "../build/_deps/moteus-src/lib/cpp/mjbots/moteus/moteus.h"

// int main(int argc, char** argv) {
//   using namespace mjbots;

//   // The following DefaultArgProcess is an optional call.  If made,
//   // then command line arguments will be handled which allow setting
//   // and configuring the default 'transport' to be used if none is
//   // specified in Controller::Options::transport.
//   moteus::Controller::DefaultArgProcess(argc, argv);

//   // There are many possible options to set for each controller
//   // instance.  Here we re-set the ID to the default (1), just to show
//   // how it is done.
//   moteus::Controller::Options options;
//   options.id = 1;
//   options.query_format.trajectory_complete = moteus::kInt8;
//   options.position_format.accel_limit = moteus::kFloat;
//   options.position_format.maximum_torque = moteus::kFloat;

//   moteus::Controller controller(options);

//   // Command a stop to the controller in order to clear any faults.
//   controller.SetStop();

//   while (true) {
//     moteus::PositionMode::Command cmd;

//     // Here we will just command a position of NaN and a velocity of
//     // 0.0.  This means "hold position wherever you are".
//     cmd.position = 0.9;
//     // cmd.velocity = 0.2;
//     cmd.accel_limit = 0.01;
//     cmd.maximum_torque = NaN;

//     const auto maybe_result = controller.SetPosition(cmd);
//     if (maybe_result) {
//       const auto r = maybe_result->values;
// ::printf("%3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   traj=%s\n",
//        static_cast<int>(r.mode),
//        r.position,
//        r.velocity,
//        r.torque,
//        r.voltage,
//        r.temperature,
//        r.fault,
//       r.trajectory_complete? "true" : "false");
// ::fflush(stdout);
//       if ((r.mode == moteus::Mode::kPosition)  &&
//         r.trajectory_complete) {
//         break;
//       }
//     }

//     // Sleep 20ms between iterations.  By default, when commanded over
//     // CAN, there is a watchdog which requires commands to be sent at
//     // least every 100ms or the controller will enter a latched fault
//     // state.
//     ::usleep(10000);
//   }
// }