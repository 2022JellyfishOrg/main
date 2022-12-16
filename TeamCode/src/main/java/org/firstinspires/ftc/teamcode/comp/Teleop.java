package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class Teleop extends LinearOpMode {
    ElapsedTime endgameTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime isToggled = new ElapsedTime();
    DcMotor backLeft, backRight, frontLeft, frontRight, lift1, lift2;


    boolean liftActive = false;
    int ticks = 0;
    double clawToggle, armPos;
    boolean isBackward = true;
    boolean isMacro = true;


    Servo claw, arm;
    BNO055IMU imu;
    int turnfac;


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


        waitForStart();

        endgameTimer.reset();
        isToggled.reset();
        liftTimer.reset();

        while (opModeIsActive()) {
            if (liftTimer.milliseconds() > 500 && liftActive) {
                if (isMacro) {
                    arm.setPosition(armPos);
                } else {
                    lift1.setTargetPosition(ticks);
                    lift2.setTargetPosition(ticks);
                    lift1.setPower(0.6);
                    lift2.setPower(0.6);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                liftActive = false;
            }

            // Drivetrain movement
            double mult;
            if (gamepad2.right_trigger == 0) {
                mult = 1;
            } else if (gamepad2.right_trigger <= 0.4 && gamepad2.right_trigger > 0) {
                mult = 2;
            } else if (gamepad2.right_trigger <= 0.8 && gamepad2.right_trigger > 0.4) {
                mult = 3;
            } else {
                mult = 4;
            }
            double y = gamepad1.left_stick_y * Constants.denominator * mult;// Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1 * Constants.denominator * mult;// Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * Constants.denominator * mult; // turning

            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx);
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx);


            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (lift1.getCurrentPosition() <= 0 || lift2.getCurrentPosition() <= 0) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if (gamepad2.left_stick_y < 0 && lift1.getCurrentPosition() >= -50) {
                lift1.setTargetPosition(lift1.getCurrentPosition() - 60);
                lift2.setTargetPosition(lift2.getCurrentPosition() - 60);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad2.left_stick_y > 0 && lift1.getCurrentPosition() <= Constants.highLift + 50) {
                lift1.setTargetPosition(lift1.getCurrentPosition() + 60);
                lift2.setTargetPosition(lift1.getCurrentPosition() + 60);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // field centric:

           /* double botHeading = -imu.getAngularOrientation().firstAngle + 90;

            double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Constants.denominator is the largest motor power (absolute value) or 1
            // This ensuclawToggle all the powers maintain the same ratio, but only when
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

   */

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

            // lift macros
            if (gamepad2.dpad_down) {
                lift1.setTargetPosition(Constants.lowLift);
                lift2.setTargetPosition(Constants.lowLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
                Constants.denominator = 0.2;
            } else if (gamepad2.dpad_left) {
                lift1.setTargetPosition(Constants.mediumLift);
                lift2.setTargetPosition(Constants.mediumLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
                isMacro = true;
                Constants.denominator = 0.2;
            } else if (gamepad2.dpad_up) {
                lift1.setTargetPosition(Constants.highLift);
                lift2.setTargetPosition(Constants.highLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
                isMacro = true;
                Constants.denominator = 0.2;
            } else if (gamepad2.dpad_right) {
                liftTimer.reset();
                ticks = 0;
                arm.setPosition(Constants.armBackwardPos);
                liftActive = true;
                isMacro = false;
                Constants.denominator = 0.6;
            } else if (gamepad2.left_bumper) {
                if (lift1.getCurrentPosition() >= 100) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() - 100);
                    lift2.setTargetPosition(lift2.getCurrentPosition() - 100);
                    lift1.setPower(Constants.upSpeed);
                    lift2.setPower(Constants.upSpeed);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftActive = true;
                }
            } else if (gamepad2.right_bumper) {
                if (lift1.getCurrentPosition() <=(Constants.highLift)) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() + 100);
                    lift2.setTargetPosition(lift2.getCurrentPosition() + 100);
                    lift1.setPower(Constants.upSpeed);
                    lift2.setPower(Constants.upSpeed);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftActive = true;
                }

                // endgame alert
                if (endgameTimer.seconds() >= 85) {
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                }
                telemetry.update();
            }
        }
    }
}