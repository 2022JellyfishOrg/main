package org.firstinspires.ftc.teamcode.comp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp
public class TeleOpV3 extends LinearOpMode {
    ElapsedTime endgameTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime isToggled = new ElapsedTime();
    ElapsedTime armActive = new ElapsedTime();
    DcMotorEx backLeft, backRight, frontLeft, frontRight, lift1, lift2;
    double mult;
    int pos = 90;
    ElapsedTime posTimer = new ElapsedTime();

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

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        lift1.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        endgameTimer.reset();
        isToggled.reset();
        liftTimer.reset();

        while (opModeIsActive()) {
            lift2.setVelocity(lift1.getVelocity());

            if (liftTimer.milliseconds() > 400 && liftActive) {
                if (isMacro) {
                    if (pos == 90) {
                        arm.setPosition(Constants.armSidewayPos);
                    } else {
                        arm.setPosition(Constants.armForwardPos);
                    }
                }
                liftActive = false;
            }
             else if (liftTimer.milliseconds() > 100 && liftActive) {
                if (!isMacro) {
                    lift1.setTargetPosition(ticks);
                    lift1.setPower(0.8);
                    lift2.setPower(0.8);
                    lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                liftActive = false;
            }
            telemetry.addData("New Pos: ", pos);
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
            if (gamepad2.left_stick_y > 0 && lift1.getCurrentPosition() >= -50) {
                lift1.setTargetPosition(lift1.getCurrentPosition() - 100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Thread.sleep(50);
            } else if (gamepad2.left_stick_y < 0 && lift1.getCurrentPosition() <= Constants.highLift + 50) {
                lift1.setTargetPosition(lift1.getCurrentPosition() + 100);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Thread.sleep(50);
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
            if (gamepad2.left_bumper) {
                if (posTimer.milliseconds() > 500) {
                    if (pos == 90) {
                        pos = 180;
                    } else {
                        pos = 90;
                    }
                    telemetry.addData("new pos: ", pos);
                    telemetry.update();
                    posTimer.reset();
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

            // arm buttons
            if (gamepad2.a) {
                if (armActive.milliseconds() > 400) {
                    double factor = (Constants.armBackwardPos - arm.getPosition())/2;
                    for (int i = 0; i < 5; i++) {
                        arm.setPosition(arm.getPosition() + factor);
                        factor /= 2;
                    }
                    arm.setPosition(Constants.armBackwardPos);
                    armActive.reset();
                }
            } else if (gamepad2.b) {
                if (armActive.milliseconds() > 400) {
                    double factor = (Constants.armSidewayPos - arm.getPosition())/2;
                    for (int i = 0; i < 5; i++) {
                        arm.setPosition(arm.getPosition() + factor);
                        factor /= 2;
                    }
                    arm.setPosition(Constants.armSidewayPos);
                    armActive.reset();
                }
            } else if (gamepad2.y) {
                if (armActive.milliseconds() > 400) {
                    double factor = (Constants.armForwardPos - arm.getPosition())/2;
                    for (int i = 0; i < 5; i++) {
                        arm.setPosition(arm.getPosition() + factor);
                        factor /= 2;
                    }
                    arm.setPosition(Constants.armForwardPos);
                    armActive.reset();
                }
            } else if (gamepad2.x) {
                if (armActive.milliseconds() > 600) {
                    double factor = (Constants.armAutonMedPos - arm.getPosition())/2;
                    for (int i = 0; i < 5; i++) {
                        arm.setPosition(arm.getPosition() + factor);
                        factor /= 2;
                    }
                    arm.setPosition(Constants.armAutonMedPos);
                    armActive.reset();
                }
            }
            // lift macros
            if (gamepad2.dpad_down) {
                lift1.setTargetPosition(Constants.lowLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
            } else if (gamepad2.dpad_left) {
                lift1.setTargetPosition(Constants.mediumLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_up) {
                lift1.setTargetPosition(Constants.highLift);
                lift1.setPower(Constants.upSpeed);
                lift2.setPower(Constants.upSpeed);
                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_right) {
                liftTimer.reset();
                ticks = 0;
                arm.setPosition(Constants.armBackwardPos);
                liftActive = true;
                isMacro = false;
            }
            // endgame alert
            if (endgameTimer.seconds() >= 85) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
            telemetry.update();
            telemetry.addData("motorPower", frontLeft.getPower());
            telemetry.update();
        }
    }
}

