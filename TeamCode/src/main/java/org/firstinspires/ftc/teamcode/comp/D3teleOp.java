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
    TouchSensor limitSwitch;
    boolean atZero = true;
    int pos = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());
            drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
            drive.update();

            if (gamepad1.left_bumper) {
                drive.backwardArm();
                drive.liftToPosition(0);
                atZero = true;
            }
            if (gamepad1.right_bumper) {
                if (registerRightBumper.milliseconds() > 300) {
                    drive.counter++;
                    registerRightBumper.reset();
                    atZero = false;
                }
            }
            if (!atZero) {
                drive.liftToPosition(drive.D3RightBumper());
            }

            if ((gamepad1.a && !Constants.lastA)) {
                drive.clawToggle();
            }
            Constants.lastA = gamepad1.a;

            if (limitSwitch.isPressed()) {
                drive.turnOffLift();
                drive.resetLiftEncoders();
            }
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("ARM POSITION: ", pos);
            telemetry.update();
        }
    }
}


