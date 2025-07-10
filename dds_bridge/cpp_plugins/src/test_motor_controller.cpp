// src/test_motor_controller.cpp

#include "../include/motor_controller.h"
#include <iostream>
#include <array>
#include <thread>
#include <chrono>

int main()
{
    try
    {
        // 1) Instantiate controller
        MotorController mc;
        std::cout << "[Main] MotorController constructed.\n";

        // 2) Launch status‐printing thread
        std::atomic<bool> status_running{true};
        std::thread status_thread([&]()
                                  {
      while (status_running.load()) {
        auto const& st = mc.getMotorStatuses();  // directly read the array
        ::printf("M0 m%3d p/v/t=(%7.3f,%7.3f,%7.3f) \t M1 m%3d p/v/t=(%7.3f,%7.3f,%7.3f)\r",
             static_cast<int>(st[0].mode),
             st[0].position,
             st[0].velocity,
             st[0].torque,
             static_cast<int>(st[1].mode),
             st[1].position,
             st[1].velocity,
             st[1].torque);
      ::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
      } });

        mc.SetMotorPosition(0.0, 5.0, 0);
        mc.SetMotorPosition(0.0, 5.0, 1);
std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        // 3) Define two poses
        constexpr size_t N = MotorController::getMOTORS();
        std::array<double, N> pose1{M_PI, -M_PI};
        std::array<double, N> pose2{-M_PI, M_PI};

        // // Helper to enqueue and wait
        // auto runPose = [&](const std::array<double,N>& target, double accel){
        //   std::cout << "[Main] Enqueuing pose: ";
        //   for (auto v : target) std::cout << v << " ";
        //   std::cout << "\n";

        //   mc.SetMotorPositions(target, accel);
        //   // busy-wait until the background worker flags completion
        //   while (!mc.getTrajectoryComplete()) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //   }
        //   std::cout << "[Main] Pose reached.\n";
        // };

        // // 4) Run first pose, then return pose
        // runPose(pose1, /*accel=*/2.0);
        // runPose(pose2, /*accel=*/2.0);

        while (true)
        {
            mc.SetMotorPositions(pose1, 5.0);
            while(!mc.getTrajectoryComplete());
            mc.SetMotorPositions(pose2, 5.0);
            while(!mc.getTrajectoryComplete());
        }

        // 5) Tear down
        status_running.store(false);
        status_thread.join();
        // std::cout << "[Main] Done. Exiting.\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "\n[Error] Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}