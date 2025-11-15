package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Autonomous", group = "LinearOpMode")
public class RedAuto extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        stopMoving();

        waitForStart(); // Ensure OpMode only runs after "Init" and "Start"

        double currentVoltage = getBatteryVoltage();
        double targetVoltage = 12.5;
        double compensation = targetVoltage / currentVoltage;
        double basePower = -1.0;
        double compensatedPower = basePower * compensation;
        shooterWheel.setPower(compensatedPower);

        shooterHand.setPosition(0);
        sleep(400);

        // Shoot three balls
        for (int i = 0; i < 3; i++) {
            sleep(2500);
            shootOneBall();
        }

        shooterHand.setPosition(0);
        sleep(500);

        // Move backward for 1 second
        moveY(-0.3);
        sleep(1000);

        // Move right for 3 seconds
        moveX(0.3);
        sleep(3000);

        stopMoving();
    }
}
