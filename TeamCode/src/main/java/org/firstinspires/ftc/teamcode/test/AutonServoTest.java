package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutonServoTest extends LinearOpMode {
    Servo arm;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.servo.get("arm");
        waitForStart();

        while (opModeIsActive()) {
            arm.setPosition(0.15);
            Thread.sleep(5000);
            arm.setPosition(0);
            Thread.sleep(5000);
            arm.setPosition(0.15);
            Thread.sleep(5000);
        }
    }

}


