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
public class D3teleOp extends LinearOpMode {
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    ElapsedTime registerY = new ElapsedTime();
    ElapsedTime registerLeftButton = new ElapsedTime();
    ElapsedTime registerX = new ElapsedTime();
    ElapsedTime waitArm = new ElapsedTime();
    ElapsedTime registerB = new ElapsedTime();
    String driveType = "robot";
    TouchSensor limitSwitch;

    boolean atStart = true;
    boolean isReset = true;
    boolean atZero = true;
    int pos = 180;
    int mult = 1;
    int sideConeCount = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        drive.counter = 0;
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (isReset) {
                drive.resetLiftEncoders();
            }
            isReset = false;

            if (gamepad1.left_stick_button) {
                if (registerLeftButton.milliseconds() > 300) {
                    if (driveType.equals("robot")) {
                        driveType = "field";
                    } else {
                        driveType = "robot";
                    }
                }
            }
            if (driveType.equals("robot")) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                gamepad1.right_stick_x/2
                        )
                );
            } else {
                Pose2d poseEstimate = drive.getPoseEstimate();
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-poseEstimate.getHeading());
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x/2
                        )
                );
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (gamepad1.right_trigger > 0) {
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() + 60);
                sleep(100);

            } else if (gamepad1.left_trigger > 0) {
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() - 60);
                sleep(100);
            }
            if (gamepad1.left_bumper) {
                if (registerLeftBumper.milliseconds() > 300) {
                    if (!atZero) {
                        drive.liftDip(300);
                        drive.backwardArm();
                        atZero = true;
                        drive.counter = 0;
                    }
                }
            }
            if (gamepad1.right_bumper) {
                atStart = false;
                if (registerRightBumper.milliseconds() > 300) {
                    drive.counter++;
                    if (drive.getLiftPos() > drive.D3RightBumper()) {
                        Constants.liftSpeed = 0.55;
                    } else {
                        Constants.liftSpeed = 0.8;
                    }
                    drive.clawClose();
                    registerRightBumper.reset();
                    atZero = false;
                    waitArm.reset();
                    telemetry.addData("atZero", atZero);

                }
            }
            if (!atZero) {
                telemetry.addData("ARM SHOULD'VE MOVED", drive.getLiftPos());
                telemetry.addData("position", pos);
                if (waitArm.milliseconds() > 600 && !atStart) {
                    drive.liftToPosition(drive.D3RightBumper());
                }
                if (waitArm.milliseconds() > 1200 && !atStart) {
                    if (pos == 90) {
                        drive.sidewayArm();
                    } else {
                        drive.forwardArm();
                    }
                    waitArm.reset();
                }

            } else {
                if (waitArm.milliseconds() > 700 && !atStart) {
                    drive.backwardArm();
                }
                if (waitArm.milliseconds() > 1200 && !atStart) {
                    drive.liftToPosition(0);
                    waitArm.reset();
                }
            }

            if ((gamepad1.a && !Constants.lastA)) {
                drive.clawToggle();
            }
            Constants.lastA = gamepad1.a;

            if (gamepad1.b) {
                if (registerB.milliseconds() > 300) {
                    if (pos == 90) {
                        pos = 180;
                    } else {
                        pos = 90;
                    }
                }
            }
            if (gamepad1.y) {
                if (registerY.milliseconds() > 300) {
                    drive.turn(Math.toRadians(180) - 1e-6);
                    registerY.reset();
                }
            }

            if (gamepad1.x) {
                if (registerX.milliseconds() > 300) {
                    int desiredPos = SampleMecanumDrive.heights[SampleMecanumDrive.heights.length - sideConeCount - 1];
                    drive.liftToPosition(desiredPos);
                }
            }
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ARM POSITION: ", pos);
            telemetry.update();
        }
    }
}


