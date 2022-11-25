package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.compfiles.Detector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class rrRight_1_2 extends LinearOpMode {

    // Declaring hardware

    // For lift method
    int notApplicable = 0;

    // claw open/closed values
    double openClaw = 0.53;
    double closedClaw = 1;

    // arm open/closed values
    int backArm = 125;
    int frontArm = 0;
    int cycles = 2;

    // vision position detection
    private static int signalZonePos;

    // Roadrunner coordinates:
    public static int startPoseX = 34;
    public static int startPoseY = -61;
    public static int depositX = 24;
    public static int depositY = -12;
    public static int loadX = 59;
    public static int loadY = -12;
    public static int parkX = 10;
    public static int parkY = depositY;

    // Declaring trajectories
    Trajectory toPreload1;
    Trajectory toPreload2;
    Trajectory toLoad;
    Trajectory toDeposit;
    Trajectory toPark;

    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing.");

        // WEBCAM STUFF
        OpenCvWebcam webcam;
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

        // start position of robot (NEEDS TO BE ACCURATE TO ABOUT AN INCH OR LESS)
        Pose2d startPose = new Pose2d(startPoseX, startPoseY, 0);

        // setting robot "drive" position to the start position above
        drive.setPoseEstimate(startPose);

        toPreload1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(depositX + 10, depositY))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toPreload2))
                .build();
        toPreload2 = drive.trajectoryBuilder(toPreload1.end())
                .lineTo(new Vector2d(depositX, depositY))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLoad))
                .build();
        toLoad = drive.trajectoryBuilder(toPreload2.end())
                .lineTo(new Vector2d(loadX, loadY))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toDeposit))
                .build();
        toDeposit = drive.trajectoryBuilder(toLoad.end())
                .lineTo(new Vector2d(depositX, depositY))
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(toLoad))
                .build();
        toPark = drive.trajectoryBuilder(toDeposit.end())
                .lineTo(new Vector2d(parkX, parkY))
                .build();

        waitForStart();

        signalZonePos = detector.getPosition();
        telemetry.addData("zone", signalZonePos);
        telemetry.update();

        // move lift to high (0 is reset, 1 is low, 2 is medium, 3 is high))
        // ifCone represents if lift is picking up auton cone or depositing (true: auton cone, false: depositing)
        drive.liftConfig(3, false);

        // rotate arm to back
        drive.armPresets(backArm);

        // follow path to preload
        drive.followTrajectory(toPreload1);
        drive.followTrajectory(toPreload2);

        // CYCLES FOR "cycles" amount of times
        for (int i = 0; i < cycles; i++) {

            // open claw
            Thread.sleep(250);
            drive.clawOpen();

            // reset arm
            drive.armPresets(frontArm);

            // reset lift to auton cone height
            drive.liftConfig(notApplicable, true);

            // drive to auton cones
            drive.followTrajectory(toLoad);
            drive.whileMotorsActive();

            // grab cone
            Thread.sleep(250);
            drive.clawClose();

            // moving cone up BEFORE following path
            drive.liftConfig(3, false);
            Thread.sleep(250);

            // start rotating arm
            drive.armPresets(backArm);

            // Go back to deposit
            drive.followTrajectory(toDeposit);
            drive.whileMotorsActive();

            SampleMecanumDrive.countCones++;


            // LOOP ENDS
        }

        Thread.sleep(250);
        drive.clawOpen();


        drive.followTrajectory(toPark);
        drive.liftConfig(0, false);


        drive.clawClose();

    }


}


