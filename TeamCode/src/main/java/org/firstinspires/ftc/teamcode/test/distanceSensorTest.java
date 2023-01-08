package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class distanceSensorTest extends LinearOpMode {
    DistanceSensor distanceSensor;
    @Override
    public void runOpMode() throws InternalError {

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double distanceToWall = distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("DISTANCE TO WALL: ", distanceToWall);
            telemetry.update();
        }

    }
}