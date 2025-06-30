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
        mc.SetPosition(0.0, 1.0, 0);
        mc.SetPosition(0.0, 1.0, 1);
        std::cout << "✔ SetPosition completed\n\n";
        sleep(1);
        mc.ReleaseArmTorque();
        sleep(0.1);
        // 3) Test full‐arm pose
        // double poses[MotorController::MOTORS] = {
        //     0.1, 0.2, 0.3, 0.4, 0.5, 0.6
        // };
        double poses_pi[MotorController::getMOTORS()] = {3.14, -3.14};
        double poses_zero[MotorController::getMOTORS()] = {0.0, 0.0};
        double accel = 5.0;
        while (true)
        {
            // std::cout << "→ Calling SetArmPose("<<poses_pi[0]", "<<", accel="<<0.5<<")\n";
            // printf("Calling SetArmPose(%2.2f, %2.2f, accel=%2.2f)\n", poses_pi[0], poses_pi[1], 0.5);
            
            mc.SetArmPose(poses_pi, accel);
            // std::cout << "✔ SetArmPose completed\n";
            sleep(0.1);
            
            // printf("Calling SetArmPose(%2.2f, %2.2f, accel=%2.2f)\n", poses_zero[0], poses_zero[1], 0.5);
            mc.SetArmPose(poses_zero, accel);
            sleep(0.1);
            // mc.ReleaseArmTorque();
        }
        sleep(0.1);
        // std::cout << "Releasing Motor Torque\n";
        mc.ReleaseArmTorque();
    }
    catch (const std::exception &e)
    {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
