package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

@TeleOp
public class teleOp extends LinearOpMode {
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime posTimer = new ElapsedTime();
    TouchSensor limitSwitch;
    int mult;
    int pos = 90;
    boolean liftActive = false;
    boolean isMacro = true;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.right_trigger == 0) {
                mult = 1;
            } else if (gamepad1.right_trigger <= 0.4 && gamepad1.right_trigger > 0) {
                mult = 2;
            } else if (gamepad1.right_trigger <= 0.8 && gamepad1.right_trigger > 0.4) {
                mult = 3;
            } else {
                mult = 4;
            }
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * mult,
                    -gamepad1.left_stick_x * mult
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x * mult
                    )
            );

            // Update everything. Odometry. Etc.
            drive.update();
            if (liftTimer.milliseconds() > 700 && liftActive) {
                if (isMacro) {
                    if (pos == 90) {
                        drive.sidewayArm();
                    } else {
                        drive.forwardArm();
                    }
                }
                liftActive = false;
            }
            else if (liftTimer.milliseconds() > 250 && liftActive) {
                if (!isMacro) {
                   drive.liftToPosition(0);
                }
                liftActive = false;
            }

            if (gamepad2.left_stick_y > 0 && drive.getLiftPos() >= -50) {
                drive.liftToPosition(drive.getLiftPos() - 100);
                Thread.sleep(150);
            } else if (gamepad2.left_stick_y < 0 && drive.getLiftPos() <= Constants.highLift + 50) {
                drive.liftToPosition(drive.getLiftPos() + 100);
                Thread.sleep(150);
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
            if (gamepad2.dpad_down) {
                drive.liftToPosition(Constants.lowLift);
                liftActive = true;
            } else if (gamepad2.dpad_left) {
                drive.liftToPosition(Constants.mediumLift);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_up) {
                drive.liftToPosition(Constants.highLift);
                liftActive = true;
                isMacro = true;
            } else if (gamepad2.dpad_right) {
                drive.backwardArm();
                isMacro = false;
            }

            if ((gamepad1.a && !Constants.lastA)) {
               drive.clawToggle();
            }
            Constants.lastA = gamepad1.a;

            if (gamepad2.a) {
                drive.forwardArm();
            } else if (gamepad2.b) {
                drive.sidewayArm();
            } else if (gamepad2.y) {
                drive.backwardArm();
            }
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ARM POSITION: ", pos);
            telemetry.update();
        }
    }
}


