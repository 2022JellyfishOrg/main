// changes to make
// move more to load

package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class leftAuton extends LinearOpMode {

    TrajectorySequence toPreload;
    TrajectorySequence toLoad;
    TrajectorySequence toDeposit;
    TrajectorySequence toPark;
    DistanceSensor loadSensor;
    DistanceSensor depositSensor;

    public static int [] coneHeights = {0, 70, 136, 195, 258};

    // used to localize properly
    double distanceToXSensor = 6.25; // TUNE TUNE TUNE
    double distanceToYSensor = -6.75;
    int signalZonePos = 1;
    double yPos = -10;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        loadSensor = hardwareMap.get(DistanceSensor.class, "loadSensor");
        depositSensor = hardwareMap.get(DistanceSensor.class, "depositSensor");
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(180));


        drive.setPoseEstimate(startPose);


        TrajectoryVelocityConstraint slowVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(15);

        TrajectoryVelocityConstraint fastVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint fastAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(30);

        toPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, -12), slowVelocityConstraint, slowAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(-26, yPos), Math.toRadians(0), slowVelocityConstraint, slowAccelerationConstraint)
                .addTemporalMarker(0.5, () -> drive.sidewayArm())
                .build();
        toLoad = drive.trajectorySequenceBuilder(toPreload.end())
                .lineTo(new Vector2d(-57, yPos), fastVelocityConstraint, fastAccelerationConstraint)
                .addTemporalMarker(0.5, () -> drive.clawOpen())
                .build();
        toDeposit = drive.trajectorySequenceBuilder(toLoad.end())
                .lineToConstantHeading(new Vector2d(-24.5, yPos))
                .addTemporalMarker(0.5, () -> drive.sidewayArm())
                .build();
        if (signalZonePos == 1) {
            toPark = drive.trajectorySequenceBuilder(toDeposit.end())
                    .forward(32)
                    .build();
        } else if (signalZonePos == 2) {
            toPark = drive.trajectorySequenceBuilder(toDeposit.end())
                    .forward(4)
                    .build();
        } else {
            toPark = drive.trajectorySequenceBuilder(toDeposit.end())
                    .back(12)
                    .build();
        }



        drive.clawOpen();

        waitForStart();

        drive.resetLiftEncoders();
        drive.clawClose();
        sleep(400);
        drive.liftConfig("high");

        drive.followTrajectorySequence(toPreload);

        for (int i = 0; i < 4; i++) {
            sleep(200);
            drive.liftDip(300);

            drive.backwardArm();
            sleep(300);
            drive.clawOpen();
            sleep(150);
            if (i == 0) {
                drive.liftToPosition(258);
            } else if (i == 1) {
                drive.liftToPosition(195);
            } else if (i == 2) {
                drive.liftToPosition(136);
            }else if (i == 3) {
                drive.liftToPosition(70);
            }

            drive.followTrajectorySequence(toLoad);

            // RELOCALIZATION
            Pose2d myXPose = drive.getPoseEstimate();
            double xDistanceVal = loadSensor.getDistance(DistanceUnit.INCH);
            double xCoord = -72 + xDistanceVal + distanceToXSensor;
            if(xCoord>=-53) xCoord = -56;
            drive.setPoseEstimate(new Pose2d(xCoord,myXPose.getY(), myXPose.getHeading()));
            telemetry.addData("xCoord: ", xCoord);
            telemetry.update();

            drive.clawClose();
            sleep(300);

            drive.liftConfig("high");
            sleep(200);
            drive.followTrajectorySequence(toDeposit);
            Pose2d myYPose = drive.getPoseEstimate();
            double yDistanceVal = depositSensor.getDistance(DistanceUnit.INCH);
            double yCoord = -yDistanceVal + distanceToYSensor;
            if (yDistanceVal >= 5) {
                yCoord = myYPose.getY();
            }
            drive.setPoseEstimate(new Pose2d(myYPose.getX(),yCoord, myYPose.getHeading()));
        }
    }
}