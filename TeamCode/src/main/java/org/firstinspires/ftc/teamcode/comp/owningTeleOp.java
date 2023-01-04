package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

@TeleOp
public class owningTeleOp extends LinearOpMode {

    // register for each button (gotta be a better way but it works...)
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    ElapsedTime registerLeftButton = new ElapsedTime();
    ElapsedTime registerX = new ElapsedTime();
    ElapsedTime registerB = new ElapsedTime();
    ElapsedTime registerRightButton = new ElapsedTime();
    ElapsedTime registerA = new ElapsedTime();

    ElapsedTime[] registers =
            {registerA, registerB, registerX, registerLeftBumper,
            registerRightBumper, registerLeftButton, registerRightButton};

    ElapsedTime waitArm = new ElapsedTime(); // used for delays in right/left bumper

    String driveType = "robot"; // either robot or field, this is a toggle
    String turnSpeed = "slow"; // either slow or fast, this is a toggle
    String clawVal = "open"; // either open or closed, this is a toggle
    boolean atStart = true; // used because you don't want timers to start until you've started an action
    boolean atZero = true; // differentiates whether its a left or right button click, and does stuff accordingly
    int pos = 180; // either 90 or 180
    int sideConeCount = 1; // index of element (really the second cone because it goes from 0-4)
    int mult = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // object used in basically everything
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // DT runs without encoder
        drive.setPoseEstimate(PoseStorage.currentPose); // knowing our orientation can be helpful in field centric
        waitForStart();
        drive.counter = 3; // default high (1 is low, 2 is med, 3 is high)
        drive.resetLiftEncoders();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.left_stick_button) {
                /* whenever it shows "milliseconds = 300" it means the amount of delay between each button click
                e.g. 300 means if you click once and then click again before 300 ms, it won't register it.
                a higher delay means u can't click fast to make many changes quick
                a lower delay means if you accidentally hold too long, you will get more clicks than you want
                gotta find the middle ground */
                if (registerLeftButton.milliseconds() > 300) {
                    if (driveType.equals("robot")) {
                        driveType = "field"; // field centric
                    } else {
                        driveType = "robot"; // robot centric
                    }
                }
            }
            if (gamepad1.right_stick_button) {
                if (registerRightButton.milliseconds() > 300) {
                    if (turnSpeed.equals("slow")) {
                        turnSpeed = "fast"; // fast turning
                    } else {
                        turnSpeed = "slow"; // slow turning
                    }
                }
            }
            if (driveType.equals("robot")) {
                if (turnSpeed.equals("slow")) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y / 2 * mult,
                                    -gamepad1.left_stick_x / 2 * mult,
                                    gamepad1.right_stick_x * 0.4 // 3x slow factor (default)
                            )
                    );
                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    gamepad1.right_stick_x // no 3x factor
                            )
                    );
                }
            } else { // field centric
                Pose2d poseEstimate = drive.getPoseEstimate();
                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ).rotated(-poseEstimate.getHeading());
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x / 3
                        )
                );
            }


            // Update odom to update pos
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (gamepad1.left_trigger > 0.3) {
                mult = 2;
            } else {
                mult = 1;
            }

            // manual lift up (change line 115, value 60 to something else if its too slow or fast)
            if (gamepad1.right_trigger > 0.3) {
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() + 60);
                sleep(100);

            }
            // 300 is delay for button (
            if (gamepad1.left_bumper) {
                if (registerLeftBumper.milliseconds() > 300) {
                    if (!atZero) { // if you are ready to go up (not going to zero)
                        drive.liftDip(300);
                        drive.prevBackArm(); // don't move arm until you are lower than a certain point to prevent getting stuck on shoulder
                        atZero = true; // ready to go down
                        drive.counter = 0;
                    }
                }
            }
            if (gamepad1.right_bumper) {
                if (atStart) {
                    drive.resetAllTimers(registers);
                }
                atStart = false;
                if (registerRightBumper.milliseconds() > 300) {
                    drive.counter++;
                    if (drive.getLiftPos() > drive.D3RightBumper()) {
                        Constants.liftSpeed = 0.8;
                    } else {
                        Constants.liftSpeed = 0.8;
                    }
                    drive.clawClose();
                    registerRightBumper.reset();
                    atZero = false;
                    waitArm.reset();
                    telemetry.addData("atZero", atZero);
                } else if (gamepad1.y) {
                    if (registerRightBumper.milliseconds() > 300) {
                        drive.counter = 3;
                        if (drive.getLiftPos() > drive.D3RightBumper()) {
                            Constants.liftSpeed = 0.8;
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
            }
            if (!atZero) {
                telemetry.addData("ARM SHOULD'VE MOVED", drive.getLiftPos());
                telemetry.addData("position", pos);
                if (waitArm.milliseconds() > 500 && !atStart) {
                    drive.liftToPosition(drive.D3RightBumper());
                }
                if (waitArm.milliseconds() > 1000 && !atStart) {
                    if (pos == 90) {
                        drive.sidewayArm();
                    } else {
                        drive.forwardArm();
                    }
                    waitArm.reset();
                }

            } else {
                if (drive.getLiftPos() < 480 && !atStart) {
                    drive.backwardArm();
                }
                if (drive.getLiftPos() > 1000 && !atStart) {
                    if (waitArm.milliseconds() > 1500 && !atStart) {
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                } else if (drive.getLiftPos() > 600 && !atStart) {
                    if (waitArm.milliseconds() > 1800 && !atStart) {
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                } else {
                    if (waitArm.milliseconds() > 2100 && !atStart) {
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                }
            }

            if (gamepad1.a) {
                if (registerA.milliseconds() > 300) {
                    if (clawVal.equals("open")) {
                        drive.clawClose();
                        clawVal = "closed";
                    } else {
                        drive.clawOpen();
                        clawVal = "open";
                    }
                }
            }

            if (gamepad1.b) {
                if (registerB.milliseconds() > 250) {
                    if (pos == 90) {
                        pos = 180;
                    } else {
                        pos = 90;
                    }
                }
            }
            if (gamepad1.x) {
                if (registerX.milliseconds() > 400) {
                    int desiredPos = SampleMecanumDrive.heights[SampleMecanumDrive.heights.length - sideConeCount - 1];
                    drive.liftToPosition(desiredPos);
                    if (sideConeCount < 4) {
                        sideConeCount++;
                    } else {
                        sideConeCount = 1;
                    }
                }
                registerX.reset();
            }
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ARM POSITION: ", pos);
            telemetry.update();
        }
    }
}


