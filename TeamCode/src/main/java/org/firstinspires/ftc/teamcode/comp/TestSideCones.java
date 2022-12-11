package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class TestSideCones extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime killSwitch = new ElapsedTime();
    DcMotor backLeft, backRight, frontLeft, frontRight, lift1, lift2;

    Servo claw, arm;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        // sensor = hardwareMap.get(ColorRangeSensor. class, "sensor");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);


        double res;

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Retrieve your pose
            if (lift1.getCurrentPosition() == 0) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if (gamepad2.left_stick_y < 0 && lift1.getCurrentPosition() <= Constants.liftTicks * Constants.highLift + 50) {
                if (lift1.getPower() == 0) {
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                if (lift1.getCurrentPosition() <= Constants.liftTicks * Constants.highLift + 100) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() + 15);
                    lift2.setTargetPosition(lift2.getCurrentPosition() + 15);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.left_stick_y > 0 && lift1.getCurrentPosition() >= -50) {
                if (lift1.getPower() == 0) {
                    lift1.setPower(1);
                    lift2.setPower(1);
                }
                if (lift1.getCurrentPosition() >= 0) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() - 15);
                    lift2.setTargetPosition(lift1.getCurrentPosition() - 15);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            telemetry.addData("liftHeight", lift1.getCurrentPosition());
            telemetry.update();
        }
    }
}