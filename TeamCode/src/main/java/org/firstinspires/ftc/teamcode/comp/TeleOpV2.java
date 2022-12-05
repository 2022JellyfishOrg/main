package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class TeleOpV2 extends LinearOpMode {
    ElapsedTime time = new ElapsedTime();
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
        time.reset();

        while (opModeIsActive()) {
            // Retrieve your pose
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

            if (lift1.getCurrentPosition() < Constants.liftTicks * Constants.lowLift) {
                Constants.turnDenom = 1;
            } else if (lift1.getCurrentPosition() < Constants.liftTicks * Constants.mediumLift) {
                Constants.turnDenom = 0.3;
            } else if (lift1.getCurrentPosition() < Constants.liftTicks * Constants.highLift) {
                Constants.turnDenom = 0.3;
            } else {
                Constants.turnDenom = 0.2;
            }





            double y = gamepad1.left_stick_y * Constants.denominator;// Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1 * Constants.denominator; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x * Constants.turnDenom; // turning

            double frontLeftPower = (y + x + rx);
            double backLeftPower = (y - x + rx);
            double frontRightPower = (y - x - rx);
            double backRightPower = (y + x - rx);


            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);





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

            int factor = 10;
            double temp;
            if (gamepad2.a) {
                /*
                for (int i = 0; i <= factor; i++) {
                    temp = arm.getPosition();
                    if (temp > 0.03) {
                        arm.setPosition(0.03);
                    } else {
                        arm.setPosition(temp - (temp * (1.0 / factor)));
                    }
                }

                 */
                arm.setPosition(Constants.armForwardPos);
            } else if (gamepad2.x) {
                arm.setPosition(Constants.armSidewayPos);
                Thread.sleep(1000);
            } else if (gamepad2.y) {
                /*
                for (int i = 0; i <= factor; i++) {
                    if (arm.getPosition() >= Constants.armBackwardPos) {
                        break;
                    }
                    temp = arm.getPosition();
                    if (temp < 0.09) {
                        arm.setPosition(0.09);
                    } else {
                        arm.setPosition(temp + (temp * (1.0 / factor)));
                    }
                }
                 */
                arm.setPosition(Constants.armBackwardPos);
            }



                /*
            } else if (gamepad2.x) {
                double temp = arm.getPosition();
                arm.setPosition(0.06);
                int sleep = (int)(Math.abs(temp - 0.06) * 10000);
                Thread.sleep(sleep);
            } else if (gamepad2.y) {
                double temp = arm.getPosition();
                arm.setPosition(0.12);
                int sleep = (int)(Math.abs(temp - 0.12) * 10000);
                Thread.sleep(sleep);
            }

                 */


            // lift macros
            if (gamepad2.dpad_down) {
                double speed = 0.8;
                // ticks = (ticks per inch)(# of inches)
                //claw.setPosition(1);

                int ticks = (int) (Constants.liftTicks * Constants.lowLift);


                lift1.setPower(speed);
                lift2.setPower(speed);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //claw.setPosition(1);
                Constants.denominator = 0.35;

            } else if (gamepad2.dpad_left) {
                double speed = 0.8;
                //claw.setPosition(1);
                // ticks = (ticks per inch)(# of inches)
                int ticks = (int) (Constants.liftTicks * Constants.mediumLift);

                lift1.setPower(speed);
                lift2.setPower(speed);

                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claw.setPosition(1);

                Constants.denominator = 0.35;
            } else if (gamepad2.dpad_up) {
                double speed = 0.8;
                //claw.setPosition(1);
                // ticks = (ticks per inch)(# of inches)
                int ticks = (int) (Constants.liftTicks * Constants.highLift);


                lift1.setPower(speed);
                lift2.setPower(speed);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claw.setPosition(1);

                Constants.denominator = 0.25;
                // opening the claw

                // resetting claw to closed

            } else if (gamepad2.dpad_right) { // resetting lift
                // ticks = (ticks per inch)(# of inches)
                int ticks = 0;
                arm.setPosition(Constants.armBackwardPos);
                Thread.sleep(1000);
                lift1.setPower(0.25);
                lift2.setPower(0.25);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Constants.denominator = 0.5;
                // opening the claw

            } else if (gamepad2.right_bumper) {
                // ticks = (ticks per inch)(# of inches)
                int ticks = 0;
                lift1.setPower(0.25);
                lift2.setPower(0.25);

                Thread.sleep(250);
                lift1.setTargetPosition(ticks);
                lift2.setTargetPosition(ticks);

                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Constants.denominator = 0.5;
            } else if (gamepad2.left_bumper) {
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
                Constants.denominator = 0.35;
            }

            if (time.seconds() >= 85) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }

        }
    }
}