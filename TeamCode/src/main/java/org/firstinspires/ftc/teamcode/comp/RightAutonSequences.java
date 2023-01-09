// changes to make
// move more to load

package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    DistanceSensor distanceSensor;

    public static int [] coneHeights = {0, 70, 136, 195, 258};

    // used to localize properly

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Pose2d startPose = new Pose2d(35.5, -61, 0);
        Pose2d preloadPose = new Pose2d(28.5,-9, Math.toRadians(-60));
        Pose2d depositPose = new Pose2d(27, -8.75, Math.toRadians(-60));
        Pose2d loadPose = new Pose2d(57, -8.5, 0);

        drive.setPoseEstimate(startPose);

        toPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(36, -20))
                .splineToSplineHeading(preloadPose, Math.toRadians(150))
                .build();
        toLoad = drive.trajectorySequenceBuilder(depositPose)
                .lineToSplineHeading(new Pose2d(45, -12, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(54.4,-8), Math.toRadians(0))
                .addTemporalMarker(0.5, () -> drive.clawOpen())
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

            telemetry.addData("coneHeight: " , drive.getLiftPos());

            drive.clawClose();
            sleep(300);

            drive.liftConfig("high");
            sleep(200);
            drive.followTrajectorySequence(toDeposit);
        }
    }
}