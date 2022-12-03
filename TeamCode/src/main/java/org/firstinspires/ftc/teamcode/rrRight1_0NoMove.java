package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;

import java.util.ArrayList;

@Autonomous
public class rrRight1_0NoMove extends LinearOpMode {

    // vision position detection
    private static int signalZonePos;

    // Roadrunner coordinates:
    public static int startPoseX = 37;
    public static int startPoseY = -61;
    public static int parkX = 34 + (signalZonePos - 1) * 24;
    public static int parkY = -30;

    // Declaring trajectories
    Trajectory toAlign;
    Trajectory toPark;
    org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    AprilTagDetection tagOfInterest = null;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    public int location = 0;
    public int getLoc(){
        return this.location;
    }


    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        // WEBCAM STUFF
        OpenCvWebcam webcam;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        //org.firstinspires.ftc.teamcode.auton.

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
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        if(tag.id==LEFT){
                            location = 0;

                        }else if(tag.id==MIDDLE){
                            location = 1;

                        }else if(tag.id==RIGHT){
                            location = 2;
                        }
                        break;
                    }
                }

            }


            telemetry.update();
            sleep(20);
        }

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(startPoseX, startPoseY, 0);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        toAlign = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(startPoseX, parkY))
                .build();
        toPark = drive.trajectoryBuilder(toAlign.end())
                .lineTo(new Vector2d(parkX, parkY))
                .build();

        waitForStart();

        signalZonePos = getLoc();
        telemetry.addData("zone", signalZonePos);
        telemetry.update();

//        drive.followTrajectory(toAlign);
//        drive.followTrajectory(toPark);

    }


}


