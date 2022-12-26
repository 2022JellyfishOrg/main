package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp
public class TeleOpForEthanSMH extends LinearOpMode {
    ElapsedTime bumperDelay = new ElapsedTime();
    ElapsedTime clawWait = new ElapsedTime();
    ColorSensor colorSensor;
    TouchSensor limitSwitch;
    DistanceSensor distanceSensor;

    DcMotorEx backLeft, backRight, frontLeft, frontRight, lift1, lift2;
    Encoder liftEncoder;

    double mult;

    int targetTicks = 0; // will end up being VERY large (about the reciprocal of coefficient)
    double coefficient = 1.0 / (8192 * 2.5);
    // conversion from ticks to power has to be TINY, because you have 8192 ticks per revolution

    Servo claw, arm, servo;
    BNO055IMU imu;

    int count = 0; // counter for right_bumper clicks
    boolean reset = true; // true means its going to zero, false means its going to a height

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        limitSwitch = hardwareMap.touchSensor.get("limitSwitch");
        liftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br"));
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setDirection(DcMotorEx.Direction.REVERSE);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        liftEncoder.setDirection(Encoder.Direction.REVERSE);



        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("targetTicks", targetTicks);
            telemetry.addData("liftHeight", liftEncoder.getCurrentPosition());
            telemetry.addData("red", colorSensor.red());
            telemetry.addData("blue", colorSensor.blue());
            telemetry.addData("limit switch", limitSwitch.isPressed());
            telemetry.addData("distance sensor", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            // Drivetrain movement
            if (gamepad1.right_trigger == 0) {
                mult = 1;
            } else if (gamepad1.right_trigger <= 0.4 && gamepad1.right_trigger > 0) {
                mult = 2;
            } else if (gamepad1.right_trigger <= 0.8 && gamepad1.right_trigger > 0.4) {
                mult = 3;
            } else {
                mult = 4;
            }

            double y = gamepad1.left_stick_y * Constants.denominator * mult;// Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1 * Constants.denominator * mult;// Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * Constants.denominator * mult;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.right_bumper) {
                if (bumperDelay.milliseconds() > 150) {
                    count++;
                    bumperDelay.reset();
                    if (count % 3 == 1) {
                        liftGoPosition(Constants.lowLift);
                    } else if (count % 3 == 2) {
                        liftGoPosition(Constants.mediumLift);
                    } else {
                        liftGoPosition(Constants.highLift);
                    }
                    arm.setPosition(Constants.armForwardPos);
                }
            }
            if (gamepad1.left_bumper) {
                int currentPos = (lift1.getCurrentPosition() + lift2.getCurrentPosition())/2;
                if (bumperDelay.milliseconds() > 150) {
                    count = 0;
                    bumperDelay.reset();
                    liftGoPosition(currentPos - 50);
                    Thread.sleep(400); // yes this isn't async but im tired (jk i'll make it better later)
                    claw.setPosition(0.4); // open
                    liftGoPosition(currentPos);
                    Thread.sleep(400);
                    arm.setPosition(Constants.armBackwardPos);
                    Thread.sleep(200);
                    liftGoPosition(0);

                }

            }

            if (lift1.getCurrentPosition() < 25) {
                lift1.setPower(0);
                lift2.setPower(0);
            } else if (limitSwitch.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }
    }
    public void liftGoPosition(int ticks) {
        lift1.setTargetPosition(ticks);
        lift2.setTargetPosition(ticks);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

