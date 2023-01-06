package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
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
public class cycleTeleOp extends LinearOpMode {

    // register for each button (gotta be a better way but it works...)
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    ElapsedTime registerLeftButton = new ElapsedTime();
    ElapsedTime registerX = new ElapsedTime();
    ElapsedTime registerB = new ElapsedTime();
    ElapsedTime registerRightButton = new ElapsedTime();
    TouchSensor limitSwitch;


    ElapsedTime waitArm = new ElapsedTime(); // used for delays in right/left bumper

    String driveType = "robot"; // either robot or field, this is a toggle
    String turnSpeed = "slow"; // either slow or fast, this is a toggle
    String clawVal = "open"; // either open or closed, this is a toggle
    boolean atStart = true; // used because you don't want timers to start until you've started an action
    boolean atZero = true; // differentiates whether its a left or right button click, and does stuff accordingly
    boolean begin = true;
    int pos = 145; // either 90 or 145
    int sideConeCount = 1; // index of element (really the second cone because it goes from 0-4)
    int mult = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // object used in basically everything
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // DT runs without encoder
        drive.setPoseEstimate(PoseStorage.currentPose); // knowing our orientation can be helpful in field centric
        waitForStart();
        drive.counter = 3; // default high (1 is low, 2 is med, 3 is high)
        drive.resetLiftEncoders();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (begin) {
                drive.clawOpen();
                begin = false;
            }

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
                drive.setWeightedDrivePower(
                        new Pose2d(
                                (gamepad1.left_stick_y / 2) * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger),
                                (gamepad1.left_stick_x / 2) * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger),
                                -gamepad1.right_stick_x / 2 * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger)// 3x slow factor (default)
                        )
                );
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

            // manual lift up (change line 115, value 60 to something else if its too slow or fast)
            if (gamepad1.dpad_up) {
                Constants.liftSpeed = 1;
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
                atStart = false;
                if (registerRightBumper.milliseconds() > 300) {
                    drive.counter = 3; // high
                    if (drive.getLiftPos() > drive.D3RightBumper()) {
                        Constants.liftSpeed = 1;
                    } else {
                        Constants.liftSpeed = 1;
                    }
                    drive.clawClose(); // claw close at 0 seconds
                    registerRightBumper.reset(); // allow for new right bumper input
                    atZero = false; // lift ready to go up
                    waitArm.reset(); // starts timer for arm
                    telemetry.addData("atZero", atZero);

                }
            } else if (gamepad1.y) { // same code as right bumper, but incremental instead of high
                if (registerRightBumper.milliseconds() > 300) {
                    drive.counter++;
                    if (drive.getLiftPos() > drive.D3RightBumper()) {
                        Constants.liftSpeed = 1;
                    } else {
                        Constants.liftSpeed = 1;
                    }
                    drive.clawClose();
                    registerRightBumper.reset();
                    atZero = false;
                    waitArm.reset();
                    telemetry.addData("atZero", atZero);

                }
            }
            if (!atZero) { // if right bumper (lift going up)
                telemetry.addData("ARM SHOULD'VE MOVED", drive.getLiftPos());
                telemetry.addData("position", pos);
                if (waitArm.milliseconds() > 450 && !atStart) { // 500 ms after claw, the lift goes up
                    drive.liftToPosition(drive.D3RightBumper());
                }
                if (drive.getLiftPos() > 230 && !atStart) { // 1000 ms after claw, arm moves
                    if (pos == 90) {
                        drive.sidewayArm();
                    } else {
                        drive.arm145(); // change to drive.forwardArm() in owning (bc its 90 and 180 not 90 and 145).
                    }
                    waitArm.reset(); // reset timer at the end so that it can take in new input from waitArm
                }

            } else {
                int dipPos = Constants.highLift - 250;
                if (drive.getLiftPos() < dipPos && !atStart) { // if lift pos is less than 1050 ticks, move from diagonal arm to backward arm
                    drive.backwardArm();
                }
                if (waitArm.milliseconds() > 1300 && !atStart) { // Wait 1300 ms from when left bumper was clicked to reset claw back to normal open, rather than COMPLETELY open
                    drive.clawOpen();
                }
                if (drive.getLiftPos() > 1000 && !atStart) { // if lift is at high wait 1200 ms
                    if (waitArm.milliseconds() > 1400 && !atStart) {
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                } else if (drive.getLiftPos() > 600 && !atStart) { // if lift is at med wait 1500
                    if (waitArm.milliseconds() > 1400 && !atStart) {
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                } else {
                    if (waitArm.milliseconds() > 2000 && !atStart) { // if lift is at low wait 1700
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                }
            }

            if ((gamepad1.a && !Constants.lastA)) {
                drive.clawToggle();
            }
            Constants.lastA = gamepad1.a;

            if (gamepad1.b) { // change between 90 and 180 (add to function above to make it automatic)
                if (registerB.milliseconds() > 250) {
                    if (pos == 90) {
                        pos = 145;
                    } else {
                        pos = 90;
                    }
                }
            }
            if (gamepad1.x) { // side cones (needs adjusting it sucks rn)
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
            telemetry.addData("ARM POSITION: ", pos);
            if (limitSwitch.isPressed()) {
                drive.resetLiftEncoders();
                telemetry.addData("ZERO POS", drive.getLiftPos());
            }
            telemetry.update();
        }
    }

}

