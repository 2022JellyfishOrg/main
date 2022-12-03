package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class LeftAuton extends LinearOpMode {
    double wheelDiameter = 3.77953;
    double circumference = wheelDiameter * Math.PI * 1/1.95;
    double lowLift = 15;
    double mediumLift = 23;
    double highLift = 33;
    double circumferenceLift = 2 * Math.PI;
    double ticksPerRevolution = 537.6;
    int liftTicks = (int) (250 / circumferenceLift);
    double speedMod = 0.4;
    int signalZonePos = 2;

    DcMotor backLeft;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor frontRight;
    DcMotor lift;
    Servo claw;

    OpenCvCamera webcam;



    @Override
    public void runOpMode() throws InterruptedException {
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

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        lift = hardwareMap.get(DcMotor.class, "lift");
        claw = hardwareMap.get(Servo.class, "claw");

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        signalZonePos = detector.getPosition();
        telemetry.addData("zone", signalZonePos);
        telemetry.update();


        /*
        switch (detector.getLocation()) {
            case LEFT: signalZonePos = 0;
            break;
            case RIGHT: signalZonePos = 1;
            break;
            case NOT_FOUND: signalZonePos = 2;

        }
        webcam.stopStreaming();
    */


        claw.setPosition(1);
        Thread.sleep(2500);
        toSignalZone(signalZonePos);





    }



    public void liftConfig(int ticks) throws InterruptedException {
        lift.setTargetPosition(ticks);
        lift.setPower(1);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lift.isBusy()) {

        }



    }

    public void toSignalZone(int pos) throws InterruptedException{
        if (pos == 1) {
            //forward(2);
            left (16);
            turnLeft(4);
            backward(13);
        }
        else if (pos == 2) {
            //forward(2);

            left(16);
        } else {
            //forward(2);
            left(16);
            turnLeft(3);
            forward(12);
        }
    }

    public void forward(double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy() || frontLeft.getCurrentPosition() >= ticks) {
            // empty while loop to hold the code
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
        }
        speedMod = 0.25;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;



    }

    public void backward (double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;


    }

    public void right (double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;


    }

    public void left (double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;


    }

    public void forwardRightDiag(double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        frontLeft.setTargetPosition(ticks);
        // backLeft.setTargetPosition(ticks);
        // frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        //frontRight.setPower(speedMod);
        //backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;


    }

    public void turnLeft (int degrees) {
        double rotation = Math.toRadians(degrees) / circumference;
        // needs adjusting
        int ticks = (int) (rotation * ticksPerRevolution * 7);
        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;

    }

    public void turnRight (int degrees) {
        double rotation = Math.toRadians(degrees) / circumference;
        // needs adjusting
        int ticks = (int) (rotation * ticksPerRevolution * 7);
        frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backRight.setTargetPosition(-ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setPower(speedMod);
        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        backRight.setPower(speedMod);
        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            if (frontLeft.getCurrentPosition() > 0.8 * frontLeft.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontLeft.getCurrentPosition() > 0.9 * frontLeft.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontLeft.getCurrentPosition() > frontLeft.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;

    }

    public void forwardLeftDiag(double dist) {
        double rotation = dist / circumference;
        // number of rotations to go distance
        int ticks = (int) (rotation * ticksPerRevolution);
        // frontLeft.setPower(speedMod
        ;        frontRight.setPower(speedMod);
        backLeft.setPower(speedMod);
        // backRight.setPower(speedMod
        ;        // frontLeft.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        // backRight.setTargetPosition(ticks);
        //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (frontRight.isBusy() || backLeft.isBusy()) {
            if (frontRight.getCurrentPosition() > 0.8 * frontRight.getTargetPosition()) {
                speedMod = 0.15;
            } else if (frontRight.getCurrentPosition() > 0.9 * frontRight.getTargetPosition()) {
                speedMod = 0.1;
            } else if (frontRight.getCurrentPosition() > frontRight.getTargetPosition()) {
                speedMod = -0.05;
            }
            // empty while loop to hold the code
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        speedMod = 0.25;



    }
}


