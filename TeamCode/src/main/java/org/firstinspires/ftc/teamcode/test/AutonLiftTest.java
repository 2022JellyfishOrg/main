package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class AutonLiftTest extends LinearOpMode {
    DcMotor lift1;
    DcMotor lift2;
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        claw = hardwareMap.servo.get("claw");
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        double res = 0;
        while (opModeIsActive()) {
            if ((gamepad2.a && !Constants.lastA)) {
                Constants.direction = !Constants.direction;
                if (!Constants.direction) {
                    res = Constants.openClaw;
                } else {
                    res = Constants.closedClaw;
                }
                claw.setPosition(res);
            }
            Constants.lastA = gamepad1.a;

            if (gamepad1.a) {
                lift1.setPower(0.8);
                lift2.setPower(0.8);
                lift1.setTargetPosition(lift1.getCurrentPosition() + 30);
                lift2.setTargetPosition(lift2.getCurrentPosition() + 30);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(50);

            } else if (gamepad1.b) {
                lift1.setPower(0.8);
                lift2.setPower(0.8);
                lift1.setTargetPosition(lift1.getCurrentPosition() - 30);
                lift2.setTargetPosition(lift2.getCurrentPosition() - 30);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(50);
            } else {
                lift1.setTargetPosition(lift1.getCurrentPosition());
                lift2.setTargetPosition(lift2.getCurrentPosition());
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("lift1Pos", lift1.getCurrentPosition());
            telemetry.addData("lift2Pos", lift2.getCurrentPosition());
            telemetry.update();
        }

    }
}



