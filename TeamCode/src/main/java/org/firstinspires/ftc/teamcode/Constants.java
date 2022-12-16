package org.firstinspires.ftc.teamcode;

import org.openftc.apriltag.AprilTagDetection;

public class Constants {

    // public means it can be accessed by the other classes
    // static means you can call the variables from the class (without reference object)
    // final means the value is constant, can't be changed
    // after that comes the object type
    // then the name of the object, then the value after the equal sign

    public static final double circumferenceLift = 2 * Math.PI; // lift pulley circumference
    public static final int liftTicks = (int) (250 / circumferenceLift);  // variable value because of inconsistencies

    public static final int cycles = 3; // number of auton cycles
    public static double turnDenom = 0.8; // turn factor in tele
    public static double denominator = 0.5; // drive factor in tele

    // claw positions
    public static final double openClaw = 0;
    public static final double closedClaw = 1;

    // arm positions
    public static final double offset = 0;
    public static final double armForwardPos = 0.255 + offset;
    public static final double armSidewayPos = 0.3125 + offset;
    public static final double armBackwardPos = 0.365 + offset;
    public static final double armAutonMedPos = armForwardPos + (armSidewayPos - armForwardPos)/2;

    // Side cones
    public static int countCones = 5; // number of side cones (changes value)
    public static final double distancePerCone = 0.35; // distance in lift inches per side cone
    public static int sideConeLift = (int) (liftTicks * (1.5 + countCones * distancePerCone)); // calculation for lift height


    // Claw booleans
    public static boolean lastA = false;
    public static boolean direction = true;

    // ninja mode booleans
    public static boolean lastBumper = false;
    public static boolean directionDenominator = true;


    // Lift heights
    public static final int inspectionLift = 300;
    public static final int lowLift = 529;
    public static final int mediumLift = 765;
    public static final int sevenInches = 300;
    public static final int highLift = 1140;
    public static double liftSpeed = 0.8;
    public static double upSpeed = 0.9;
    public static double downSpeed = 0.5;

    // Apriltag stuff
    public static AprilTagDetection tagOfInterest;
    public static final double fx = 578.272;
    public static final double fy = 578.272;
    public static final double cx = 402.145;
    public static final double cy = 221.506;
    public static final double tagsize = 0.166;

    // Vision choice
    public static final int LEFT = 1;
    public static final int MIDDLE = 2;
    public static final int RIGHT = 3;

    // Roadrunner coordinates
    /*
    public static double startPoseX = 34;
    public static double startPoseY = -61;
    public static double depositX = 32;
    public static double depositY = -11;
    public static double startAngle = Math.toRadians(0);
    public static double depositAngle = Math.toRadians(0);
    public static double loadAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);
    public static double loadX = 57.5;
    public static double loadY = -10;
    public static double parkX = 34 + 24 * (location - 1);
    public static double parkY = depositY;
    */
    public static double startPoseX = 34;
    public static double startPoseY = -61;
    public static double depositX = 33;
    public static double depositY = -14;
    public static double startAngle = Math.toRadians(0);
    public static double depositAngle = Math.toRadians(0);
    public static double loadAngle = Math.toRadians(0);
    public static double parkAngle = Math.toRadians(0);
    public static double loadX = 57;
    public static double loadY = -12;
    public static double parkX = 33;
    public static double parkY = -14;
    public static double leftStartPoseX = -34;
    public static double leftStartPoseY = -61;
    public static double leftDepositX = -33;
    public static double leftDepositY = -14;
    public static double leftStartAngle = Math.toRadians(180);
    public static double leftDepositAngle = Math.toRadians(180);
    public static double leftLoadAngle = Math.toRadians(180);
    public static double leftParkAngle = Math.toRadians(180);
    public static double leftLoadX = -57;
    public static double leftLoadY = -12;
    public static double leftParkX = -33;
    public static double leftParkY = -14;
}