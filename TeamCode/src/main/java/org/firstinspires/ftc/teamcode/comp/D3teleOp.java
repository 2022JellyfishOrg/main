package org.firstinspires.ftc.teamcode.comp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;

@TeleOp
public class D3teleOp extends LinearOpMode {
    ElapsedTime registerRightBumper = new ElapsedTime();
    ElapsedTime registerLeftBumper = new ElapsedTime();
    TouchSensor limitSwitch;
    ElapsedTime waitArm = new ElapsedTime();
    boolean atZero = true;
    int pos = 90;
    int mult = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            if (gamepad1.right_trigger == 0) {
                mult = 1;
            } else if (gamepad1.right_trigger <= 0.4 && gamepad1.right_trigger > 0) {
                mult = 2;
            } else if (gamepad1.right_trigger <= 0.8 && gamepad1.right_trigger > 0.4) {
                mult = 3;
            } else {
                mult = 4;
            }
            Vector2d input = new Vector2d(-gamepad1.left_stick_y * mult, -gamepad1.left_stick_x * mult).rotated(-poseEstimate.getHeading());
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * mult));
            drive.update();

            if (gamepad1.left_bumper) {
                if (registerLeftBumper.milliseconds() > 300) {
                    drive.backwardArm();
                    atZero = true;
                }
            }
            if (gamepad1.right_bumper) {
                if (registerRightBumper.milliseconds() > 300) {
                    drive.liftToPosition(drive.D3RightBumper());
                    drive.counter++;
                    registerRightBumper.reset();
                    atZero = false;
                    waitArm.reset();
                }
            }
            if (!atZero) {
                if (waitArm.milliseconds() > 700) {
                    drive.forwardArm();
                }
            } else {
                if (waitArm.milliseconds() > 500) {
                    drive.liftToPosition(0);
                }
            }

            if ((gamepad1.a && !Constants.lastA)) {
                drive.clawToggle();
            }
            Constants.lastA = gamepad1.a;


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ARM POSITION: ", pos);
            telemetry.update();
        }
    }
}


