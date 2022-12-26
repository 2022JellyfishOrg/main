package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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

    Trajectory toPreload1;
    Trajectory toPreload2;
    Trajectory toLoad;
    Trajectory toPark;
    Trajectory toDeposit;
    TrajectoryBuilder toDepositInit;
    ElapsedTime timer = new ElapsedTime();
    OpenCvCamera webcam;
    int signalZonePos = 0;
    int offset = 0;

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

        signalZonePos = detector.getPosition();
        telemetry.addData("zone", signalZonePos);
        telemetry.update();

        Pose2d startPose = new Pose2d(35.5, -61, 0);

        drive.setPoseEstimate(startPose);
        int pos = (36 + (24 * (signalZonePos - 2)));

        toPreload1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, -20, 0))
                .addTemporalMarker(0.5, () -> drive.setArm(Constants.armForwardPos))
                .build();
        toPreload2 = drive.trajectoryBuilder(toPreload1.end())
                .lineToLinearHeading(new Pose2d(30.5, -13, Math.toRadians(-45)))
                .build();
        toLoad = drive.trajectoryBuilder(toDeposit.end())
                .lineToLinearHeading(new Pose2d(56, -12, 0))
                .addTemporalMarker(0.5, () -> drive.setArm(Constants.armBackwardPos))
                .addTemporalMarker(0.75, () -> drive.liftConfig("sideCones"))
                .build();
        toDeposit = drive.trajectoryBuilder(toLoad.end())
                .lineToLinearHeading(new Pose2d(30.5, -13, Math.toRadians(-45)))
                .addTemporalMarker(0.5, () -> drive.setArm(Constants.armForwardPos))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineToLinearHeading(new Pose2d(pos, -10, 0))
                .addTemporalMarker (0.25, () -> drive.setArm(Constants.armBackwardPos))
                .addTemporalMarker (0.5, () -> drive.liftConfig("zero"))
                .build();

        waitForStart();

        drive.clawClose();
        sleep(2000);
        drive.liftConfig("medium");
        drive.followTrajectory(toPreload1);
        drive.followTrajectory(toPreload2);
        drive.liftDip();
        drive.followTrajectory(toLoad);
        drive.clawClose();
        sleep(1000);
        drive.liftConfig ("medium");
        drive.followTrajectory(toDeposit);
        sleep(1000);
        drive.liftDip();
        drive.followTrajectory(toPark);







    }


}