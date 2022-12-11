package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class TeleOpV2 extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    DcMotor backLeft, backRight, frontLeft, frontRight, lift1, lift2;
    boolean liftActive = false;

    Servo claw, arm;
    BNO055IMU imu;
    int ticks = 0;


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

            if (liftTimer.milliseconds() > 500 && liftActive) {
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);
                lift1.setPower(0.8);
                lift2.setPower(0.8);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            double y = gamepad1.left_stick_y * Constants.denominator;// Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1 * Constants.denominator; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * Constants.denominator; // turning

            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx);
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx);


            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (lift1.getCurrentPosition() == 0) {
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if (gamepad2.left_stick_y < 0 && lift1.getCurrentPosition() <= Constants.liftTicks * Constants.highLift + 50) {
                if (lift1.getPower() == 0) {
                    lift1.setPower(0.3);
                    lift2.setPower(0.3);
                }
                if (lift1.getCurrentPosition() <= Constants.liftTicks * Constants.highLift + 100) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() + 60);
                    lift2.setTargetPosition(lift2.getCurrentPosition() + 60);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.left_stick_y > 0 && lift1.getCurrentPosition() >= -50) {
                if (lift1.getPower() == 0) {
                    lift1.setPower(0.3);
                    lift2.setPower(0.3);
                }
                if (lift1.getCurrentPosition() >= 0) {
                    lift1.setTargetPosition(lift1.getCurrentPosition() - 60);
                    lift2.setTargetPosition(lift1.getCurrentPosition() - 60);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


            }

            // field centric:

           /* double botHeading = -imu.getAngularOrientation().firstAngle + 90;

            double rotX = (x * Math.cos(botHeading) - y * Math.sin(botHeading));
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Constants.denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double Constants.denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / Constants.denominator;
            double backLeftPower = (rotY - rotX + rx) / Constants.denominator;
            double frontRightPower = (rotY - rotX - rx) / Constants.denominator;
            double backRightPower = (rotY + rotX - rx) / Constants.denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            */

            // A, A
            /*
            if (gamepad1.b) {
                Constants.denominator = 1;
            } else {
                Constants.denominator = 0.5;
            }

             */

            // manual lift (test)

            if ((gamepad1.a && !Constants.lastA)) {
                Constants.direction = !Constants.direction;
                if (!Constants.direction) {
                    res = Constants.openClaw;
                } else {
                    res = Constants.closedClaw;
                }
                claw.setPosition(res);
            }
            Constants.lastA = gamepad1.a;


            if (gamepad2.a) {
                arm.setPosition(Constants.armBackwardPos);
                Thread.sleep(1000);
            } else if (gamepad2.b) {
                arm.setPosition(Constants.armSidewayPos);
                Thread.sleep(1000);
            } else if (gamepad2.y) {
                arm.setPosition(Constants.armForwardPos);
                Thread.sleep(1000);
            } else if (gamepad2.x) {
                arm.setPosition(Constants.armAutonMedPos);
                Thread.sleep(1000);
            }
            // lift macros
            if (gamepad2.dpad_down) {
                double speed = 1;
                // ticks = (ticks per inch)(# of inches)
                //claw.setPosition(1);
                int ticks = 344;


                lift1.setPower(speed);
                lift2.setPower(speed);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //claw.setPosition(1);

            } else if (gamepad2.dpad_left) {
                double speed = 1;
                //claw.setPosition(1);
                // ticks = (ticks per inch)(# of inches)
                int ticks = 557;

                lift1.setPower(speed);
                lift2.setPower(speed);

                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claw.setPosition(1);

            } else if (gamepad2.dpad_up) {
                double speed = 1;
                //claw.setPosition(1);
                // ticks = (ticks per inch)(# of inches)
                int ticks = 786;

                lift1.setPower(speed);
                lift2.setPower(speed);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claw.setPosition(1);
                arm.setPosition(Constants.armBackwardPos);
                // opening the claw

                // resetting claw to closed

            } else if (gamepad2.dpad_right) { // resetting lift
                // ticks = (ticks per inch)(# of inches)
            //    arm.setPosition(Constants.armForwardPos);
                int ticks = 0;
              //  arm.setPosition(Constants.armBackwardPos);
                Thread.sleep(1000);
                lift1.setPower(0.6);
                lift2.setPower(0.6);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Thread.sleep(500);
                // opening the claw

            } else if (gamepad2.right_bumper) {
                // ticks = (ticks per inch)(# of inches)
                Thread.sleep(500);
                int ticks = 0;
                lift1.setPower(0.25);
                lift2.setPower(0.25);

                Thread.sleep(250);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            } else if (gamepad2.left_bumper) { // b
                double speed = 0.8;
                // ticks = (ticks per inch)(# of inches)
                //claw.setPosition(1);

                int ticks = Constants.sideConeLift;
                Constants.countCones--;


                lift1.setPower(speed);
                lift2.setPower(speed);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Thread.sleep(500);
                arm.setPosition(Constants.armForwardPos);
                //claw.setPosition(1);
            }

            // endgame alert
            if (timer.seconds() >= 85) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

            //

        }
    }
}