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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class RightAutonSequences extends LinearOpMode {

    TrajectorySequence toPreload;
    TrajectorySequence toLoad;
    TrajectorySequence toDeposit;
    TrajectorySequence toPark;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35.5, -61, 0);
        Pose2d preloadPose = new Pose2d(29.5,-8, Math.toRadians(-60));
        Pose2d depositPose = new Pose2d(27.75, -7.75, Math.toRadians(-60));
        Pose2d loadPose = new Pose2d(57, -8, 0);

        drive.setPoseEstimate(startPose);

        toPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(36, -20))
                .splineToSplineHeading(preloadPose, Math.toRadians(150))
                .addTemporalMarker(0.5, () -> drive.forwardArm())
                .build();
        toLoad = drive.trajectorySequenceBuilder(depositPose)
                .lineToSplineHeading(new Pose2d(40, -12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(54.4,-8), Math.toRadians(0))
                .addTemporalMarker(0.05, () -> drive.clawOpen())
                .build();
        toDeposit = drive.trajectorySequenceBuilder(loadPose)
                .lineToConstantHeading(new Vector2d(40, -12))
                .splineToSplineHeading(depositPose, Math.toRadians(150))
                .addTemporalMarker(0.5, () -> drive.forwardArm())
                .build();

        drive.clawClose();

        waitForStart();

        drive.resetLiftEncoders();
        drive.clawClose();
        sleep(400);
        drive.liftConfig("high");

        drive.followTrajectorySequence(toPreload);

        for (int i = 0; i < 1; i++) {
            sleep(200);
            drive.liftDip(300);

            drive.backwardArm();
            sleep(300);
            drive.liftConfig("sideCone");
            drive.followTrajectorySequence(toLoad);

            drive.clawClose();
            sleep(300);

            drive.liftConfig("high");
            sleep(200);
            drive.followTrajectorySequence(toDeposit);
        }
    }
}