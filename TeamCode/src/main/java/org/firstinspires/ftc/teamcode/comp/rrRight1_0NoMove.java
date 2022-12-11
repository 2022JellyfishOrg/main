package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class rrRight1_0NoMove extends LinearOpMode {

    // Declaring trajectories
    Trajectory toAlign;
    Trajectory toPark;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double tag = Constants.tagsize;



    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        int loc = 0;
        // WEBCAM STUFF
        OpenCvWebcam webcam;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
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
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Constants.LEFT || tag.id == Constants.MIDDLE || tag.id == Constants.RIGHT)
                    {
                        Constants.tagOfInterest = tag;
                        if(tag.id==Constants.LEFT){
                            loc = 0;

                        }else if(tag.id==Constants.MIDDLE){
                            loc = 1;

                        }else if(tag.id==Constants.RIGHT){
                            loc = 2;
                        }
                    }
                    telemetry.update();
                }
                telemetry.update();

            }
            telemetry.addData("zone", loc);
            telemetry.update();
            sleep(1000);

        }
        waitForStart();


    }


}


