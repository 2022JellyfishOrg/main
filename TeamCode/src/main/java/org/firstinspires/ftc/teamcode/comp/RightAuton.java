package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    Trajectory toDepositOffset1;
    Trajectory toDepositOffset2;
    ElapsedTime timer = new ElapsedTime();
    OpenCvCamera webcam;
    int signalZonePos = 0;
    int offset = 0;


    // Vision pipeline
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int [] heights = {155, 125, 85, 28, 0};


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class,"webby"), cameraMonitorViewId);
        Detector detector = new Detector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        waitForStart();
        signalZonePos = detector.getPosition();
        telemetry.addData("zone", signalZonePos);
        telemetry.update();

        // Setting up trajectories and start position BEFORE starting

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(Constants.startPoseX, Constants.startPoseY, Constants.startAngle);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);
        int pos = (34 + (24 * (signalZonePos - 2)));

        toPreload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(Constants.depositX+1, Constants.depositY+2, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toLoad = drive.trajectoryBuilder(toPreload.end())
                .lineToLinearHeading(new Pose2d(Constants.loadX + 1.0/2.0, Constants.loadY-1, Constants.loadAngle))
                .build();
        toDeposit = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 1.5, Constants.depositY, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toDepositOffset1 = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 1.5, Constants.depositY + offset, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toDepositOffset2 = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 1.5, Constants.depositY + 2 * offset, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineToLinearHeading(new Pose2d(pos, Constants.parkY, Constants.parkAngle))

                .build();

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
            Thread.sleep(1000);
            drive.clawOpen();
            Thread.sleep(250);

            // reset lift to auton cone height
            timer.reset();
            drive.setArm(Constants.armBackwardPos);
            drive.followTrajectory(toLoad);

            while (timer.seconds() < 2) {

            }

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
            if (Constants.cycles == 2) {
                drive.followTrajectory(toDepositOffset1);
                offset += 0.5;
            }
        }

        Thread.sleep(500);
        drive.clawOpen();
        Thread.sleep(500);

        drive.setArm(Constants.armBackwardPos);
        Thread.sleep(1000);

        //drive.followTrajectory(toPark);
        drive.liftConfig(0, false);
        drive.followTrajectory(toPark);


        drive.clawClose();
    }


}


