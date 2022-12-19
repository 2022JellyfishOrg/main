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
public class ParkLeftAuton extends LinearOpMode {

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
        Pose2d startPose = new Pose2d(-61, Constants.startPoseY, Constants.startAngle);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);
        int pos;
        if (signalZonePos == 0) {
            pos = -36;
        } else {
            pos = (-36 + (24 * (signalZonePos - 2)));
        }
/*
        toPreload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(Constants.depositX, Constants.depositY -27, Constants.depositAngle))
                .addTemporalMarker(0.25, () -> drive.liftConfig(6, false))
                .build();

 */
        toPreload = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(37, -33, Constants.parkAngle))
                .build();
        /*
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

         */
        toPark = drive.trajectoryBuilder(toPreload.end())
                .lineToLinearHeading(new Pose2d(pos, -33, Constants.parkAngle))
                .build();

        // move lift to high (0 is reset, 1 is low, 2 is medium, 3 is high))
        // ifCone represents if lift is picking up auton cone or depositing (true: auton cone, false: depositing)

        // rotate arm to back

        // follow path to preload

        /*drive.liftConfig(2, false);
        telemetry.addData("countCones1", Constants.countCones);
        telemetry.addData("ticks", SampleMecanumDrive.ticks);
        telemetry.update();

         */
        //drive.clawClose();
        //Thread.sleep(2500);
        drive.followTrajectory(toPreload);
        drive.followTrajectory(toPark);

    }


}