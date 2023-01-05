// changes to make
// move more to load

package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class RightAuton90 extends LinearOpMode {

    Trajectory toPreload1;
    Trajectory toPreload2;
    Trajectory toLoad1;
    Trajectory toLoad2;
    Trajectory toPark;
    Trajectory toDeposit;
    TrajectoryBuilder toDepositInit;
    ElapsedTime timer = new ElapsedTime();
    OpenCvCamera webcam;
    int signalZonePos = 2;
    int offset = 0;

    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /*
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
         */

        Pose2d startPose = new Pose2d(35.5, -61, 0);
        Pose2d preloadPose = new Pose2d(30.5, -8.2, Math.toRadians(-60));
        Pose2d depositPose = new Pose2d(22.5, -11, Math.toRadians(90));
        Pose2d loadPose = new Pose2d(56.5, -10, 0);

        drive.setPoseEstimate(startPose);
        int pos = (36 + (24 * (signalZonePos - 2)));

        toPreload1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(39, -18))
                .addTemporalMarker(0.5, () -> drive.setArm(Constants.armForwardPos))
                .build();
        toPreload2 = drive.trajectoryBuilder(toPreload1.end())
                .lineToLinearHeading(preloadPose)
                .build();
        toLoad1 = drive.trajectoryBuilder(preloadPose)
                .lineToLinearHeading(loadPose)
                .addTemporalMarker(0.5, () -> drive.liftConfig("sideCone"))
                .build();
        toDeposit = drive.trajectoryBuilder(loadPose)
                // .lineToLinearHeading(depositPose)
                .lineToLinearHeading(depositPose)
                .build();
        toLoad2 = drive.trajectoryBuilder(depositPose)
                // .lineToLinearHeading(depositPose)
                .lineToLinearHeading(loadPose)
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineToLinearHeading(new Pose2d(pos, -10, 0))
                .addTemporalMarker (0.5, () -> drive.liftConfig("zero"))
                .build();

        waitForStart();
        drive.resetLiftEncoders();
        drive.clawClose();
        sleep(1200);
        drive.liftConfig("high");
        drive.followTrajectory(toPreload1);
        drive.followTrajectory(toPreload2);
        for (int i = 0; i < 5; i++) {
            sleep(200);
            drive.liftDip(400);
            sleep(400);
            drive.backwardArm();
            sleep(300);
            drive.clawOpen();
            if (i == 0) {
                drive.followTrajectory(toLoad1);
            } else {
                drive.followTrajectory(toLoad2);
            }
            drive.clawClose();
            sleep(500);
            drive.liftConfig ("high");
            sleep(350);
            drive.followTrajectory(toDeposit);
            sleep(400);
            drive.liftDip(400);
        }
        drive.backwardArm();
        sleep(1000);

        // drive.followTrajectory(toPark);







    }


}