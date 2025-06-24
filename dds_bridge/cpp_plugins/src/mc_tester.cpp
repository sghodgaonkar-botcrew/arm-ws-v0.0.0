// test_motor_controller.cpp

#include "../include/motor_controller.h" // your header
#include <iostream>
#include <exception>

int main()
{
    try
    {
        // 1) Instantiate
        // MotorController mc({1.7, 1.7});
        MotorController mc;
        std::cout << "✅ MotorController constructed successfully\n";

        // 2) Test single‐motor move (will block until position or throw)
        // std::cout << "→ Calling SetPosition(0.5 rad, accel=1.0, id=0)\n";
        mc.SetPosition(0.0, 1.0, 0);
        mc.SetPosition(0.0, 1.0, 1);
        // std::cout << "✔ SetPosition completed\n";

        // 3) Test full‐arm pose
        // double poses[MotorController::MOTORS] = {
        //     0.1, 0.2, 0.3, 0.4, 0.5, 0.6
        // };
        // double poses[MotorController::getMOTORS()] = {
        // 5.0, -5.0
        // };
        // std::cout << "→ Calling SetArmPose([...], accel=0.5)\n";
        // mc.SetArmPose(poses, NaN);
        // // sleep(2);
        // std::cout << "✔ SetArmPose completed\n";

        sleep(0.1);
        // std::cout << "Releasing Motor Torque\n";
        mc.ReleaseArmTorque();
    }
    catch (const std::exception &e)
    {
        std::cerr << "❌ Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
