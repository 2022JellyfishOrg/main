//Ethan-Op, will be a combination of 180 owning and cycling, along with 145 all swappable

//CONTROLS:

//Joystick Left - Directional Movement at 50% power, faster with RT
//Joystick Right - Turning at 50% power, faster with RT

//Right Toggle - Increases movement and turning up to 100%

//Left Bumper - Dips at any height, wide claw, then resets slides and claw
//Right Bumper - 1. Grabs claw and goes to 180 and high during cycling, 2. Same thing but goes low, medium and high with more presses for owning

//A - Cone Stack, 2, 3, 4, 5, in that order repeating
//B - Toggles between to Owning and Cycling, with Cycling as the default
//X - Reset slides to 0, manually
//Y - Turn the Robot 180 degrees from where it is

//D-Up - Manual Up, in case something gets stuck
//D-Down - Manual Down, for backup reasons

//MODE - Used for moving the drivetrain in straight lines

//Not used:
//Left Toggle - Haven't found a use case for it, slow mode wasn't very helpful
//D-Right - Dpad Hard to use, could accidentally hit the up/down
//D-Left - Dpad Hard to use, could accidentally hit the up/down
//Joystick Left Press - Hard to click and can be accidentally clicked
//Joystick Right Press - Hard to click and can be accidentally clicked

//Bunch of imports and unhelpful boring stuff
package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
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
import org.firstinspires.ftc.teamcode.drive.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
//

@TeleOp
public class ethanOp extends LinearOpMode {



    //Some timers and variables
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    ElapsedTime registerA = new ElapsedTime();
    ElapsedTime registerX = new ElapsedTime();
    ElapsedTime registerB = new ElapsedTime();
    ElapsedTime registerDPAD = new ElapsedTime();
    TouchSensor limitSwitch;

    ElapsedTime waitArm = new ElapsedTime();
    String driveType = "robot";

    //Turning 180
    ElapsedTime registerY = new ElapsedTime();
    boolean setting = true;
    double target180 = 0.0;

    boolean atStart = true;
    boolean begin = true;
    boolean atHome = true; //should actually be called atHome
    boolean sideConeActive = false;
    boolean groundActive = false;

    //Cone heights
    int [] coneHeights = {0, 70, 136, 195, 258};
    int sideConeCount = 0;

    //If it's true, cycling, if not, owning
    boolean toggleSwitch = true;

    boolean isTurning = false;
    //If it equals 1, it'll alternate between low, medium, and high


    @Override
    public void runOpMode() throws InterruptedException {
        //Probably don't need to know
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TwoTrackingWheelLocalizer localizer = new TwoWheelTrackingLocalizer(hardwareMap, drive);
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        Constants.armBackwardPos = 0.98;

        drive.setPoseEstimate(PoseStorage.currentPose);
        int turnAngle = 0;

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
                                //The following starts at a base of 40%, but with each tick of the trigger, it increases the speed by 0.006, essentially allowing it to reach 100% from 40%
                                //(gamepad1.left_stick_y * 0.4) + ((100*gamepad1.right_trigger)*0.006), makes sense of this, but now we simplify:
                                (gamepad1.left_stick_y * 0.4) + (gamepad1.right_trigger*0.6),
                                (gamepad1.left_stick_x * 0.4) + (gamepad1.right_trigger*0.6),
                                //Same math here, base at 35%, but increases by 0.0065
                                (-gamepad1.right_stick_x  * 0.35) + (gamepad1.right_trigger*0.65)
                        )
                );
            } else if (driveType.equals("turn")) {
                if (!isTurning) {
                    drive.turnAsync(Math.toRadians(turnAngle)); // only wanna do it once
                    isTurning = true;
                }
                if (!drive.isBusy()) { // if it's done turning
                    driveType = "robot";
                }
            }

            // Update everything. Odometry. Etc.
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            if (drive.getLiftPos() < 15 & atHome) {
                Constants.armBackwardPos= 0.98;
            } else if (!atHome) {
                Constants.armBackwardPos = 0.97;
            }
            //Reset Lift
            if (gamepad1.left_bumper) {
                if (registerLeftBumper.milliseconds() > 300) {
                    if (!atHome) {
                        sideConeActive = false;
                        drive.liftDip(300);
                        drive.backwardArm();
                        atHome = true;
                        drive.counter = 0;
                    }
                }
            }
            //Determines cycling/owning, then closes claw, resets the timer, starts !atHome, resets the other timer, and prints
            if (gamepad1.right_bumper) {
                if (registerRightBumper.milliseconds() > 300) {
                    sideConeActive = false;
                    atStart = false;
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
            if (!atHome && !sideConeActive) {
                telemetry.addData("ARM SHOULD'VE MOVED", drive.getLiftPos());

                //REFINE HERE: Shortest Time Possible so that the claw grabs, slides move up, then the arm does
                if (drive.counter == 1 || toggleSwitch) { //This means it's been clicked one time, and slides are up, clicking it 3 more times back to low, and the value is 4, for owning
                    //Start moving to slides height determined in the first right bumper part
                    //toggleSwitch is for cycling, since  you'd want delays
                    if (waitArm.milliseconds() > 350 && !atStart) { //600
                        drive.liftToPosition(drive.D3RightBumper());
                    }
                    //Start arm, after the slides have begun moving UP
                    if (waitArm.milliseconds() > 450 && !atStart) { //800
                        drive.forwardArm();
                        waitArm.reset();
                    }
                } else { //NO MORE delays in between, for owning
                    drive.liftToPosition(drive.D3RightBumper());
                    waitArm.reset(); //idk if we need this but I though I'd reset either way
                }

            } else if (!sideConeActive) {
                sideConeCount = 0;//The next step after left bumper, these are all the wait times for different heights
                int dipPos = Constants.highLift - 250;

                if (drive.getLiftPos() < dipPos && !atStart) { // if lift pos is less than 1050 ticks, move from diagonal arm to backward arm
                    drive.backwardArm();
                }

                //This is the delay after the claw has dropped cone/been set to 0/WIDE Open, during the lift dip
                //Having a delay this long allows for the claw to fully open, and then proceed with the following actions
                //This ensure we don't knock over the junction or hit it in any way; it's also faster
                //REFINE HERE: The delay should be from the millisecond it drops to the point at which it's out of the hitting junction zone
                if (waitArm.milliseconds() > 1000 && !atStart) { //1400
                    drive.clawOpen();
                }

                //REFINE HERE: Delays until the slides start moving down, low should take the longest, high the shortest
                if (drive.getLiftPos() > 1100 && !atStart) { // HIGH
                    if (waitArm.milliseconds() > 1200 && !atStart) { //WAIT 1000 original ms, since you don't want to crash slides while arm is moving
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                } else if (drive.getLiftPos() > 600 && !atStart) { //MEDIUM
                    if (waitArm.milliseconds() > 1200 && !atStart) { //WAIT 1200 ms
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                } else { //LOW
                    if (waitArm.milliseconds() > 1600 && !atStart) { //WAIT 1600 ms, since you don't want to crash slides while arm is moving
                        drive.liftToPosition(0);
                        waitArm.reset();
                    }
                }
            }

            //Toggling between cycling and owning
            if (gamepad1.x) {
                if (registerX.milliseconds() > 300) {
                    if (toggleSwitch) { //If cycling, go to owning
                        toggleSwitch = false;
                    } else { //If owning, go to cycling
                        toggleSwitch = true;
                    }
                    registerX.reset();
                }
            }

            //Cone stack code that goes from 2-->5, and repeats
            //NOTE* AUTON AND TELE ARE NO LONGER RUNNING THE SAME CONE STACK HEIGHTS IF VALUES ARE CHANGED, AND THEY ARE, ORDER IS DIFFERENT
            if (gamepad1.a) {
                if (registerA.milliseconds() > 300) {
                    sideConeActive = true;
                    int desiredPos = coneHeights[sideConeCount];
                    if (sideConeCount == 4) { //max cone height/pressed 3 times, go back to 2nd cone height
                        sideConeCount = 0;
                    } else {
                        sideConeCount++;
                    }
                    registerA.reset();
                    drive.liftToPosition(desiredPos);
                }
            }

            //To Reset the Lift, manually
            if (limitSwitch.isPressed()) {
                drive.resetLiftEncoders();
            }

            if (gamepad1.left_trigger > 0) { //Manual Up, also sleeps a bit
                Constants.liftSpeed = 0.8;
                drive.liftToPosition(drive.getLiftPos() + 60);
                sleep(100);
            }

            if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_down) {
                if (registerDPAD.milliseconds() > 300) {
                    int initAngle = (int) Math.toDegrees(poseEstimate.getHeading());
                    isTurning = false;
                    if (gamepad1.dpad_up) {
                        turnAngle = 270 - initAngle;
                    } else if (gamepad1.dpad_left) {
                        turnAngle = 180  - initAngle;
                    } else if (gamepad1.dpad_right) {
                        turnAngle = -initAngle;
                    } else if (gamepad1.dpad_down){
                        turnAngle = 90 - initAngle;
                    }
                    registerDPAD.reset();
                }
            }

            if (gamepad1.b) {
                if (registerB.milliseconds() > 300) {
                    if (!groundActive) {
                        sideConeActive = true; // yes this isn't a side cone but it's easier to just use this variable to tell the code to overwrite the cycling/owning lift code
                        drive.clawClose();
                        sleep(300);
                        drive.liftToPosition(100);
                        groundActive = true;
                    } else {
                        drive.clawOpen();
                        drive.liftToPosition(0);
                        groundActive = false;
                    }
                }
            }



            telemetry.addData("(0 is 2nd cone) sideConeCount:", sideConeCount);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("cycling true, owning false: ", toggleSwitch);
            telemetry.update();
        }
    }
}

