package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hackerstuff.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class RightAuton extends LinearOpMode {

    /* Declaring trajectories (toPreload and toDeposit are the same
    for now just made two different trajectories for simplicity)
     */
    Trajectory toPreload;
    Trajectory toLoad;
    Trajectory toDeposit;
    Trajectory toPark;

    // Vision pipeline
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int [] heights = {103, 78, 44, 20, 0};


    public void runOpMode() throws InterruptedException {

        // declaring drive (object from which we create our path, its basically the robot)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // WEBCAM STUFF (stores position in value called "location")
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(telemetry);

        webcam.setPipeline(aprilTagDetectionPipeline);
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
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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

        // Setting up trajectories and start position BEFORE starting

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(Constants.startPoseX, Constants.startPoseY, Constants.startAngle);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        toPreload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(Constants.depositX, Constants.depositY, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toLoad = drive.trajectoryBuilder(toPreload.end())
                .lineToLinearHeading(new Pose2d(Constants.loadX, Constants.loadY, Constants.loadAngle))
                .build();
        toDeposit = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX, Constants.depositY, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineToLinearHeading(new Pose2d(Constants.parkX, Constants.parkY, Constants.parkAngle))

                .build();

        waitForStart();

        // move lift to high (0 is reset, 1 is low, 2 is medium, 3 is high))
        // ifCone represents if lift is picking up auton cone or depositing (true: auton cone, false: depositing)

        // rotate arm to back

        drive.resetLifts();
        // follow path to preload
        drive.clawClose();
        Thread.sleep(1500);

        drive.liftConfig(2, false);
        telemetry.addData("countCones1", Constants.countCones);
        telemetry.addData("ticks", SampleMecanumDrive.ticks);
        telemetry.update();
        drive.followTrajectory(toPreload);


        // CYCLES FOR "cycles" amount of times
        for (int i = 0; i < Constants.cycles; i++) {

            // open claw
            Thread.sleep(250);
            drive.clawOpen();
            Thread.sleep(250);

            // reset lift to auton cone height
            drive.setArm(Constants.armBackwardPos);

            Thread.sleep(250);

            drive.liftConfig(heights[i], true);
            Constants.countCones--;
            telemetry.addData("countCones2", Constants.countCones);
            telemetry.addData("ticks", SampleMecanumDrive.ticks);
            telemetry.update();

            // drive to auton cones
            drive.followTrajectory(toLoad);

            // grab cone
            drive.clawClose();
            Thread.sleep(250);

            // moving cone up BEFORE following path
            drive.liftConfig(2, false);

            // Go back to deposit
            drive.followTrajectory(toDeposit);
        }

        Thread.sleep(500);
        drive.clawOpen();
        Thread.sleep(500);


        //drive.followTrajectory(toPark);
        drive.liftConfig(0, false);


        drive.clawClose();
    }


}


