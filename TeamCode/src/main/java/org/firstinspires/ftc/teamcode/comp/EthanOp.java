//Ethan-Op, will be a combination of 180 owning and cycling, along with 145 all swappable

//CONTROLS:

//Joystick Left - Directional Movement at 50% power, faster with RT
//Joystick Right - Turning at 50% power, faster with RT

//Joystick Left Press - N/A (Hard to click)
//Joystick Right Press - N/A (Hard to click)

//Left Toggle - N/A
//Right Toggle - Increases movement and turning up to 100%

//Left Bumper - Dips at any height, wide claw, then resets slides and claw
//Right Bumper - 1. Grabs claw and goes to 180 and high during cycling, 2. Same thing but goes low, medium and high with more presses for owning

//A - Cone Stack, 2, 3, 4, 5, in that order repeating
//B - Toggles between to Owning and Cycling, with Cycling as the default
//X - Reset slides to 0, manually
//Y - Spin the Robot 180 degrees [TO DO]

//D-Up - Manual Up, in case something gets stuck
//D-Right - N/A
//D-Left - N/A
//D-Down - Manual Down, for backup reasons

//MODE - N/A

//Bunch of imports and unhelpful boring stuff
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
//

@TeleOp
public class EthanOp extends LinearOpMode {
    //Some timers and variables
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    ElapsedTime registerA = new ElapsedTime();
    ElapsedTime waitArm = new ElapsedTime();
    String driveType = "robot";

    boolean atStart = true;
    boolean begin = true;
    boolean atHome = true; //should actually be called atHome
    int pos = 180;
    int mult = 1;
    TouchSensor limitSwitch;

    //Cone heights
    int [] coneHeights = {70, 136, 195, 258};
    int sideConeCount = 0;

    //If it's true, cycling, if not, owning
    boolean toggleSwitch = true;
    //If it equals 1, it'll alternate between low, medium, and high

    @Override
    public void runOpMode() throws InterruptedException {
        //Probably don't need to know
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();

        //Resets count for lift and lift motor encoders
        drive.counter = 0;
        drive.resetLiftEncoders();

        //Beginning stuff, have claw open right away and run
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            if (begin) {
                drive.clawOpen();
                begin = false;
            }

            //It is "robot", see the variables of driveType right where timers are initialized
            if (driveType.equals("robot")) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                (gamepad1.left_stick_y / 2) * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger),
                                (gamepad1.left_stick_x / 2) * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger),
                                -gamepad1.right_stick_x / 2 * (1+gamepad1.right_trigger) * (1-gamepad1.left_trigger)
                        )
                );
            }
            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            //Reset Lift
            if (gamepad1.left_bumper) {
                if (registerLeftBumper.milliseconds() > 300) {
                    if (!atHome) {
                        drive.liftDip(300);
                        drive.backwardArm();
                        atHome = true;
                        drive.counter = 0;
                    }
                }
            }

            //Determines cycling/owning, then closes claw, resets the timer, starts !atHome, resets the other timer, and prints
            if (gamepad1.right_bumper) {
                atStart = false;
                if (registerRightBumper.milliseconds() > 300) {

                    //Decide between owning and cycling:
                    if (toggleSwitch) { //cycling high
                        drive.counter = 3;
                    } else { //owning/low/medium/high
                        drive.counter++;
                    }
                    Constants.liftSpeed = 1;
                    drive.clawClose();
                    registerRightBumper.reset();
                    atHome = false;
                    waitArm.reset();
                    telemetry.addData("atHome", atHome);

                }
            }

            //The next step after right bumper
            if (!atHome) {
                telemetry.addData("ARM SHOULD'VE MOVED", drive.getLiftPos());

                if (drive.counter == 1) { //This means it's been clicked one time, and slides are up, clicking it 3 more times back to low, and the value is 4, for owning
                    //Start moving to slides height determined in the first right bumper part
                    if (waitArm.milliseconds() > 600) {
                        drive.liftToPosition(drive.D3RightBumper());
                    }

                    //Start arm, after the slides have begun moving UP
                    if (waitArm.milliseconds() > 800) {
                        drive.forwardArm();
                        waitArm.reset();
                    }
                } else if (toggleSwitch) { //for cycling, since you want delays
                    //Start moving to slides height determined in the first right bumper part
                    if (waitArm.milliseconds() > 600) {
                        drive.liftToPosition(drive.D3RightBumper());
                    }

                    //Start arm, after the slides have begun moving UP
                    if (waitArm.milliseconds() > 800) {
                        drive.forwardArm();
                        waitArm.reset();
                    }
                } else { //NO MORE delays in between, for owning
                    drive.liftToPosition(drive.D3RightBumper());
                    waitArm.reset(); //idk if we need this but I though I'd reset either way
                }

            } else { //The next step after left bumper, these are all the wait times for different heights
                int dipPos = Constants.highLift - 250;

                if (drive.getLiftPos() < dipPos && !atStart) { // if lift pos is less than 1050 ticks, move from diagonal arm to backward arm
                    drive.backwardArm();
                }

                //This is the delay after the claw has dropped cone/been set to 0/WIDE Open, during the lift dip
                //Having a delay this long allows for the claw to fully open, and then proceed with the following actions
                //This ensure we don't knock over the junction or hit it in any way; it's also faster
                //The delay should be from the millisecond it drops to the point at which it's out of the hitting junction zone
                if (waitArm.milliseconds() > 1400 && !atStart) { //DECREASE THIS DELAY
                    drive.clawOpen();
                }

                //These are the times that will wait until the slides start moving down, THESE MUST BE FINE TUNED
                if (drive.getLiftPos() > 1000 && !atStart) { // HIGH
                    if (waitArm.milliseconds() > 1500 && !atStart) { //WAIT 1500 ms, since you don't want to crash slides while arm is moving
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                } else if (drive.getLiftPos() > 600 && !atStart) { //MEDIUM
                    if (waitArm.milliseconds() > 1500 && !atStart) { //WAIT 1500 ms
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                } else { //LOW
                    if (waitArm.milliseconds() > 1800 && !atStart) { //WAIT 1800 ms, since you don't want to crash slides while arm is moving
                        drive.liftToPosition(-20);
                        waitArm.reset();
                    }
                }
            }

            //Toggling between cycling and owning
            if (gamepad1.b) {
                if (toggleSwitch) { //If cycling, go to owning
                    toggleSwitch = false;
                } else { //If owning, go to cycling
                    toggleSwitch = true;
                }
            }

            //Cone stack code that goes from 2-->5, and repeats
            //NOTE* AUTON AND TELE ARE NO LONGER RUNNING THE SAME CONE STACK HEIGHTS IF VALUES ARE CHANGED, AND THEY ARE, ORDER IS DIFFERENT
            if (gamepad1.a) {
                if (registerA.milliseconds() > 300) {
                    int desiredPos = coneHeights[sideConeCount];
                    if (sideConeCount == 3) { //max cone height/pressed 3 times
                        sideConeCount = 0;
                    } else {
                        sideConeCount++;
                    }
                    registerA.reset();
                    drive.liftToPosition(desiredPos);
                }
            }

            //Reset the lift, manually, in case it's a bit off or something was pulled
            if (gamepad1.x && limitSwitch.isPressed()) {
                drive.resetLiftEncoders();
            }

            if (gamepad1.dpad_up) { //Manual Up, also sleeps a bit
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() + 60);
                sleep(100);
            } else if (gamepad1.dpad_down) { //Manual Down, also sleeps a bit
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() - 60);
                sleep(100);
            }

            telemetry.addData("(0 is 2nd cone) sideConeCount:", sideConeCount);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
