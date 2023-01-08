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
    DistanceSensor distanceSensor;

    public static int [] coneHeights = {0, 70, 136, 195, 258};

    // used to localize properly
    double distanceToSensor = 8.2; // TUNE TUNE TUNE
    int signalZonePos = 1;
    int yPos = -12;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Pose2d startPose = new Pose2d(-35, -61, Math.toRadians(180));


        drive.setPoseEstimate(startPose);


        TrajectoryVelocityConstraint slowVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint slowAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(15);

        toPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-35, -20), slowVelocityConstraint, slowAccelerationConstraint)
                .splineToConstantHeading(new Vector2d(-24, yPos), Math.toRadians(0), slowVelocityConstraint, slowAccelerationConstraint)
                .addTemporalMarker(0.5, () -> drive.armAuton())
                .build();
        toLoad = drive.trajectorySequenceBuilder(toPreload.end())
                .lineTo(new Vector2d(-56, yPos))
                .addTemporalMarker(0.5, () -> drive.clawOpen())
                .build();
        toDeposit = drive.trajectorySequenceBuilder(toLoad.end())
                .lineTo(new Vector2d(-24, yPos))
                .addTemporalMarker(0.5, () -> drive.forwardArm())
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
            Pose2d myPose = drive.getPoseEstimate();
            double distanceVal = distanceSensor.getDistance(DistanceUnit.INCH);
            double xCoord = 72 - distanceVal - distanceToSensor;
            drive.setPoseEstimate(new Pose2d(xCoord,myPose.getY(), myPose.getHeading()));
            telemetry.addData("xCoord: ", xCoord);
            telemetry.update();

            drive.clawClose();
            sleep(300);

            drive.liftConfig("high");
            sleep(200);
            drive.followTrajectorySequence(toDeposit);
        }
    }
}