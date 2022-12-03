package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutonClawTest extends LinearOpMode {
    Servo arm;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.servo.get("claw");
        waitForStart();

        while (opModeIsActive()) {
            claw.setPosition(0.15);
            Thread.sleep(5000);
            claw.setPosition(0);
            Thread.sleep(5000);
            claw.setPosition(0.15);
            Thread.sleep(5000);
        }
    }

}


