package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueAuto", group = "LinearOpMode")
//Declares as autonomous file, SDK thing
public class BlueAuto extends Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        stopMoving();

        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
        shooterHand.setPosition(0.0);  // UP position (blocks balls)
        sleep(200);                    // Let servo settle
        waitForStart();

        shooterHand.setPosition(0);
        sleep(400);

        // Now start flywheel warmup
        double shooterVelocity = -1300;
        shooterWheel.setVelocity(shooterVelocity);
        sleep(1500);  // Flywheel spins up to stable speed
        while (distanceSensor.getDistance(DistanceUnit.INCH)<37){
            moveY(-0.3);
        }

        stopMoving();


//        moveY(-0.3);
//        sleep(200);
//        stopMoving();

        // NOW safe to open gate for first ball - function starts with hand UP
        shootThreeBallsGateStyle();

        moveY(0.2);
        sleep(120);
        stopMoving();
        sleep(300);


        moveY(0.5);
        sleep(500);

        moveX(-0.5);
        sleep(1500);

        // Turn off flywheel when done
        shooterWheel.setVelocity(0);
        shooterHand.setPosition(0.5);  // UP position (blocks balls)


    }
}