package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous(name = "Straight Auto Blue", group = "LinearOpMode")
public class StraightAutoBlue extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        stopMoving();

        waitForStart(); // Ensure OpMode only runs after "Init" and "Start"

        moveY(-0.3);
        sleep(800);


        moveX(-0.4);
        sleep(2000);

        moveY(0.3);
        sleep(500);

        moveX(0.3);
        sleep(720);

        stopMoving();
    }
} 