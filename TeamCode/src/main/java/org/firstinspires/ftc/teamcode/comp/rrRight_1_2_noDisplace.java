package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class rrRight_1_2_noDisplace extends LinearOpMode {
    // Declaring trajectories
    Trajectory toPreload1;
    Trajectory toPreload2;
    Trajectory toLoad;
    Trajectory toDeposit;
    Trajectory toPark;
    public void runOpMode() throws InterruptedException {

        // WEBCAM STUFF
        OpenCvWebcam webcam;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        Constants.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(Constants.tagsize, Constants.fx, Constants.fy, Constants.cx, Constants.cy);
        //org.firstinspires.ftc.teamcode.auton.

        webcam.setPipeline(Constants.aprilTagDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = Constants.aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Constants.LEFT || tag.id == Constants.MIDDLE || tag.id == Constants.RIGHT)
                    {
                        Constants.tagOfInterest = tag;
                        tagFound = true;
                        if(tag.id== Constants.LEFT){
                            Constants.location = 0;

                        }else if(tag.id== Constants.MIDDLE){
                            Constants.location = 1;

                        }else if(tag.id== Constants.RIGHT){
                            Constants.location = 2;
                        }
                        break;
                    }
                }

            }
            telemetry.update();
            sleep(20);
        }

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(Constants.startPoseX, Constants.startPoseY, 0);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        toPreload1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(Constants.depositX + 10, Constants.depositY))
                .build();
        toPreload2 = drive.trajectoryBuilder(toPreload1.end())
                .lineTo(new Vector2d(Constants.depositX, Constants.depositY))
                .build();
        toLoad = drive.trajectoryBuilder(toPreload2.end())
                .lineTo(new Vector2d(Constants.loadX, Constants.loadY))
                .build();
        toDeposit = drive.trajectoryBuilder(toLoad.end())
                .lineTo(new Vector2d(Constants.depositX, Constants.depositY))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineTo(new Vector2d(Constants.parkX, Constants.parkY))
                .build();

        waitForStart();

        // move lift to high (0 is reset, 1 is low, 2 is medium, 3 is high))
        // ifCone represents if lift is picking up auton cone or depositing (true: auton cone, false: depositing)
        drive.liftConfig(3, false);

        // rotate arm to back
        drive.setArm(Constants.armForwardPos);

        // follow path to preload
        drive.followTrajectory(toPreload1);
        drive.followTrajectory(toPreload2);

        // CYCLES FOR "cycles" amount of times
        for (int i = 0; i < Constants.cycles; i++) {

            // open claw
            Thread.sleep(250);
            drive.clawOpen();

            // reset arm
            drive.setArm(Constants.armForwardPos);

            // reset lift to auton cone height
            drive.liftConfig(0, true);

            // drive to auton cones
            drive.followTrajectory(toLoad);

            // grab cone
            Thread.sleep(250);
            drive.clawClose();

            // moving cone up BEFORE following path
            drive.liftConfig(3, false);
            Thread.sleep(500);

            // start rotating arm
            drive.setArm(Constants.armBackwardPos);

            // Go back to deposit
            drive.followTrajectory(toDeposit);

            Constants.countCones--;


            // LOOP ENDS
        }

        Thread.sleep(250);
        drive.clawOpen();


        drive.followTrajectory(toPark);
        drive.liftConfig(0, false);


        drive.clawClose();
    }


}


