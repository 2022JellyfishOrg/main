package org.firstinspires.ftc.teamcode.rrTest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class TrajectoryTest extends LinearOpMode {

    Trajectory test;


    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        // WEBCAM STUFF
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(0, 0, 0);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        test = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();

        waitForStart();

        drive.followTrajectory(test);


    }
}


