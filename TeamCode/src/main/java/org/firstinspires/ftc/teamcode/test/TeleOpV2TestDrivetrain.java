package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOpV2TestDrivetrain extends LinearOpMode {
    double wheelDiameter = 3.77953;
    double circumference = wheelDiameter * Math.PI * 0.513;
    double circumferenceLift = 2 * Math.PI;
    double ticksPerRevolution = 537.6;
    int liftTicks = (int) (250 / circumferenceLift);  // variable value because of inconsistencies
    double turnDenom = 0.8;
    double denominator = 0.8;
    double openClaw = 0;
    double closedClaw = 0.5;
    boolean armInUse = false;
    double armForwardPos = 0;
    double armSidewayPos = 0.05;
    double armBackwardPos = 0.1;
    ElapsedTime time = new ElapsedTime();



    int signalZonePos = 0;
    boolean lastA = false;
    boolean direction = true;
    double lowLift = 7;
    double mediumLift = 13;
    double highLift = 19;
    DcMotor backLeft, backRight, frontLeft, frontRight, lift1, lift2;

    Servo claw, arm;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);


        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        // sensor = hardwareMap.get(ColorRangeSensor. class, "sensor");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            frontLeft.setPower(gamepad1.left_stick_x/3);
            backLeft.setPower(gamepad1.left_stick_y/3);
            frontRight.setPower(gamepad1.right_stick_x/3);
            backRight.setPower(gamepad1.right_stick_y/3);
        }
    }
}