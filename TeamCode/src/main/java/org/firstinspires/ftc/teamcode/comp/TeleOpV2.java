package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class TeleOpV2 extends LinearOpMode {
    ElapsedTime endgameTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime isToggled = new ElapsedTime();
    ColorSensor colorSensor;
    TouchSensor limitSwitch;
    DistanceSensor distanceSensor;
    DcMotor backLeft, backRight, frontLeft, frontRight, lift1, lift2;
    double mult;
    int pos = 90;
    ElapsedTime posTimer = new ElapsedTime();
    boolean liftActive = false;
    int ticks = 0;
    double clawToggle, armPos;
    boolean isMacro = true;
    Servo claw, arm;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        limitSwitch = hardwareMap.touchSensor.get("limitSwitch");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
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
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        endgameTimer.reset();
        isToggled.reset();
        liftTimer.reset();

        while (opModeIsActive()) {
            if (liftTimer.milliseconds() > 700 && liftActive) {
                if (isMacro) {
                    if (pos == 90) {
                        sidewayArm();
                    } else {
                        forwardArm();
                    }
                }
                liftActive = false;
            }
             else if (liftTimer.milliseconds() > 100 && liftActive) {
                if (!isMacro) {
                    liftToPos(ticks, 0.6);
                }
                liftActive = false;
            }

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
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad2.left_stick_y > 0 && lift1.getCurrentPosition() >= -50) {
                liftManual(-100);
            } else if (gamepad2.left_stick_y < 0 && lift1.getCurrentPosition() <= Constants.highLift + 50) {
                liftManual(100);
            }
            if (gamepad2.left_bumper) {
                if (posTimer.milliseconds() > 500) {
                    if (pos == 90) {
                        pos = 180;
                    } else {
                        pos = 90;
                    }
                }
            }

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

            if (gamepad2.a) {
                forwardArm();
            } else if (gamepad2.b) {
                sidewayArm();
            } else if (gamepad2.y) {
                backwardArm();
            }
            telemetry.addData("armPosition", arm.getPosition());
            telemetry.update();
            if (gamepad2.dpad_down) {
                liftToPos(Constants.lowLift, Constants.upSpeed);
                liftActive = true;
            } else if (gamepad2.dpad_left) {
                liftToPos(Constants.mediumLift, Constants.upSpeed);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_up) {
                liftToPos(Constants.highLift, Constants.upSpeed);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_right) {
                backwardArm();
            }
            if (limitSwitch.isPressed()) {
                lift1.setPower(0);
                lift2.setPower(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    public void forwardArm() {
        double pos = Constants.armForwardPos;
        arm.setPosition(pos + 0.1);
        for (int i = 0; i < 10; i++) {
            arm.setPosition(arm.getPosition() - 0.01);
        }
        arm.setPosition(pos);
    }
    public void sidewayArm() {
        double pos = Constants.armSidewayPos;
        if (armPos < pos) {
            arm.setPosition(pos - 0.1);
            for (int i = 0; i < 10; i++) {
                arm.setPosition(arm.getPosition() + 0.01);
            }
            arm.setPosition(pos);
        } else {
            arm.setPosition(pos + 0.1);
            for (int i = 0; i < 10; i++) {
                arm.setPosition(arm.getPosition() - 0.01);
            }
            arm.setPosition(pos);
        }
    }
    public void backwardArm() {
        double pos = Constants.armBackwardPos;
        arm.setPosition(pos - 0.1);
        for (int i = 0; i < 10; i++) {
            arm.setPosition(arm.getPosition() + 0.01);
        }
        arm.setPosition(pos);
    }
    public void liftToPos(int height, double speed) {
        lift1.setTargetPosition(height);
        lift2.setTargetPosition(height);
        lift1.setPower(speed);
        lift2.setPower(speed);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void liftManual (int distance) {
        lift1.setTargetPosition(lift1.getCurrentPosition() + distance);
        lift2.setTargetPosition(lift1.getCurrentPosition() + distance);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}


