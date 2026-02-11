package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "RedAuto", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class RedAuto extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        stopMoving();

        waitForStart();

        shooterHand.setPosition(0);
        sleep(400);

        while (distanceSensor.getDistance(DistanceUnit.INCH)<37){
            moveY(-0.3);
        }

        stopMoving();
//
//        moveY(-0.3);
//        sleep(200);
//        stopMoving();

        shootOneBallWithEncoderRedOutside();
        //shootOneBallWithEncoderRedOutside();

        moveY(0.2);
        sleep(120);
        stopMoving();
        sleep(900);

        shootOneBallWithEncoderLastRed();

        moveY(0.5);
        sleep(500);

        moveX(0.5);
        sleep(2000);


    }
}