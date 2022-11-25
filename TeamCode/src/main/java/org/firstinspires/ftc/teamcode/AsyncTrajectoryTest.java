package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class AsyncTrajectoryTest extends LinearOpMode {

    // Declaring hardware
    DcMotor frontLeft, frontRight, backLeft, backRight, lift1, lift2;
    Servo claw, arm;
    // Declaring trajectories
    Trajectory test1;
    Trajectory test2;

    double liftSpeed = 0.5;
    double lowLift = 15;
    double mediumLift = 23;
    double highLift = 33;
    double circumferenceLift = 2 * Math.PI;
    int liftTicks = (int) (250 / circumferenceLift);

    // counting number of auton cones to measure height
    int countCones = 0;



    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        // WEBCAM STUFF
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(0, 0, 0);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        test1 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();
        test2 = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        lift1 = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        claw = hardwareMap.get(Servo.class, "claw");

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

        waitForStart();

        drive.followTrajectory(test1);
        drive.liftConfig(3, false);
        drive.liftConfig(0, false);
        drive.followTrajectory(test2);

        drive.followTrajectory(test1);
        drive.whileMotorsActive();
        drive.liftConfig(3, false);


    }



}


