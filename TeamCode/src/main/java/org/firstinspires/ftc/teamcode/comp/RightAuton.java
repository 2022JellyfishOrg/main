package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hackerstuff.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hackerstuff.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RightAuton extends LinearOpMode {

    /* Declaring trajectories (toPreload and toDeposit are the same
    for now just made two different trajectories for simplicity)
     */
    Trajectory toPreload;
    Trajectory toLoad;
    Trajectory toPark;
    Trajectory toDeposit1;
    Trajectory toDeposit2;
    Trajectory toDeposit3;
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
        int pos = (36 + (24 * (signalZonePos - 2)));

        toPreload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(Constants.depositX+1, Constants.depositY + 1, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toLoad = drive.trajectoryBuilder(toPreload.end())
                .lineToLinearHeading(new Pose2d(Constants.loadX, Constants.loadY-0.5, Constants.loadAngle))
                .build();
        toDeposit1 = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 2.5, Constants.depositY, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toDeposit2 = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 2.5, Constants.depositY + 0.3, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toDeposit3 = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(Constants.depositX + 2.5, Constants.depositY + 0.7, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.setArm(Constants.armAutonMedPos))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit3.end())
                .lineToLinearHeading(new Pose2d(pos, Constants.parkY, Constants.parkAngle))
                .build();

        // move lift to high (0 is reset, 1 is low, 2 is medium, 3 is high))
        // ifCone represents if lift is picking up auton cone or depositing (true: auton cone, false: depositing)

        // rotate arm to back

        drive.resetLifts();
        // follow path to preload
        drive.clawClose();
        Thread.sleep(1000);

        drive.liftConfig(2, false);
        telemetry.addData("countCones1", Constants.countCones);
        telemetry.addData("ticks", SampleMecanumDrive.ticks);
        telemetry.update();
        drive.followTrajectory(toPreload);


        // CYCLES FOR "cycles" amount of times
        for (int i = 0; i < Constants.cycles; i++) {
            // open claw
            Thread.sleep(250);
            drive.liftConfig(4, false);
            Thread.sleep(500);
            drive.clawOpen();
            Thread.sleep(250);
            drive.liftConfig(2, false);
            Thread.sleep(500);

            // reset lift to auton cone height
            timer.reset();
            drive.setArm(Constants.armBackwardPos);
            drive.followTrajectory(toLoad);

            drive.liftConfig(heights[i], true);
            Thread.sleep(800);
            Constants.countCones--;
            telemetry.addData("countCones2", Constants.countCones);
            telemetry.addData("ticks", SampleMecanumDrive.ticks);
            telemetry.update();

            // grab cone
            drive.clawClose();
            Thread.sleep(300);

            drive.liftConfig(2, false);

            // drive to auton cones
            if (i == 0) {
                drive.followTrajectory(toDeposit1);
            } else if (i == 1) {
                drive.followTrajectory (toDeposit2);
            } else {
                drive.followTrajectory(toDeposit3);
            }



            // moving cone up BEFORE following path

            // Go back to deposit

        }

        Thread.sleep(250);
        drive.liftConfig(4, false);
        Thread.sleep(500);
        drive.clawOpen();
        Thread.sleep(250);
        drive.liftConfig(2, false);
        Thread.sleep(500);

        drive.setArm(Constants.armBackwardPos);
        Thread.sleep(400);

        //drive.followTrajectory(toPark);
        drive.liftConfig(0, false);
        drive.followTrajectory(toPark);

    }


}