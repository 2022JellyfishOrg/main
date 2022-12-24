// with through-bore and limit switch

package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp
public class TeleOpV2 extends LinearOpMode {
    ElapsedTime limitTimer = new ElapsedTime();
    ElapsedTime endgameTimer = new ElapsedTime();
    ElapsedTime posTimer = new ElapsedTime();

    ColorSensor colorSensor;
    TouchSensor limitSwitch;
    DistanceSensor distanceSensor;

    DcMotorEx backLeft, backRight, frontLeft, frontRight, lift1, lift2;
    Encoder liftEncoder;

    double mult;

    int armPos = 90;

    int targetTicks = 0; // will end up being VERY large (about the reciprocal of coefficient)
    double coefficient = 1.0 / (8192 * 2.5);
    // conversion from ticks to power has to be TINY, because you have 8192 ticks per revolution

    double clawToggle;
    Servo claw, arm;
    BNO055IMU imu;


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

        lift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
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
            backRight.setPower(backRightPower);// turning


            if (gamepad2.left_bumper) {
                if (posTimer.milliseconds() > 500) {
                    if (armPos == 90) {
                        armPos = 180;
                    } else {
                        armPos = 90;
                    }
                }
            }

            // claw toggle
            if ((gamepad1.a && !Constants.lastA)) {
                Constants.direction = !Constants.direction;
                if (!Constants.direction) {
                    clawToggle = Constants.openClaw;
                } else {
                    clawToggle = Constants.closedClaw;
                }
                claw.setPosition(clawToggle);
            }

            Constants.lastA = gamepad1.a;


            // arm buttons
            if (gamepad2.a) {
                arm.setPosition(Constants.armBackwardPos);
            } else if (gamepad2.b) {
                arm.setPosition(Constants.armSidewayPos);
            } else if (gamepad2.y) {
                arm.setPosition(Constants.armForwardPos);
            } else if (gamepad2.x) {
                arm.setPosition(Constants.armAutonMedPos);
            }

            int currentTicks = liftEncoder.getCurrentPosition();
            double liftPower = coefficient * (targetTicks - currentTicks);
            liftPower = Range.clip(liftPower, -0.6, 0.8);
            lift1.setPower(liftPower);
            lift2.setPower(liftPower);

            if (gamepad2.dpad_down) {
                targetTicks = 20000;
            }
            if (gamepad2.dpad_left) {
                targetTicks = 40000;
            }
            if (gamepad2.dpad_up) {
                targetTicks = 60000;
            }
            if (gamepad2.dpad_right) {
                targetTicks = 0;
            }

            if (limitSwitch.isPressed()) {
                if (limitTimer.milliseconds() > 300) {
                    lift1.setPower(0);
                    lift2.setPower(0);
                }
            }
        }
    }
}

