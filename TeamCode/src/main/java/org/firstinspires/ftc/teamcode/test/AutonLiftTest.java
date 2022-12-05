package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AutonLiftTest extends LinearOpMode {
    DcMotor lift1;
    DcMotor lift2;

    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        waitForStart();

        while (opModeIsActive()) {
            lift1.setPower(gamepad1.right_stick_y);
            lift2.setPower(gamepad1.right_stick_y);
        }
    }

}


