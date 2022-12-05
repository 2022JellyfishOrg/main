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
public class rrRight1_0NoMove extends LinearOpMode {

    // Declaring trajectories
    Trajectory toAlign;
    Trajectory toPark;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;



    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        // WEBCAM STUFF
        OpenCvWebcam webcam;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(Constants.tagsize, Constants.fx, Constants.fy, Constants.cx, Constants.cy);

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
                        if(tag.id==Constants.LEFT){
                            Constants.location = 0;

                        }else if(tag.id==Constants.MIDDLE){
                            Constants.location = 1;

                        }else if(tag.id==Constants.RIGHT){
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

        toAlign = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(Constants.startPoseX, Constants.parkY))
                .build();
        toPark = drive.trajectoryBuilder(toAlign.end())
                .lineTo(new Vector2d(Constants.parkX, Constants.parkY))
                .build();

        waitForStart();

        telemetry.addData("zone", Constants.location);
        telemetry.update();

    }


}


