package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends LinearOpMode {
    Servo arm;

    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.servo.get("arm");
        claw = hardwareMap.servo.get("claw");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Position: ", arm.getPosition());
            if (gamepad1.a) {
                if (arm.getPosition() < 0.995) {
                    arm.setPosition(arm.getPosition() + 0.01);
                    Thread.sleep(100);
                }
            } else if (gamepad1.b) {
                if (arm.getPosition() > 0.005) {
                    arm.setPosition(arm.getPosition() - 0.01);
                    Thread.sleep(100);
                }
            }
            telemetry.update();
        }
        if (gamepad1.y) {
            claw.setPosition(0);
        } else if (gamepad1.x) {
            claw.setPosition(1);
        }

    }
}

