package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.comp.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

public class Constants {
    public static double circumferenceLift = 2 * Math.PI; // lift pulley circumference
    public static int liftTicks = (int) (250 / circumferenceLift);  // variable value because of inconsistencies

    public static int cycles = 2; // number of auton cycles
    public static double turnDenom = 0.8; // turn factor in tele
    public static double denominator = 0.8; // drive factor in tele

    // claw positions
    public static double openClaw = 0;
    public static double closedClaw = 1;

    // arm positions
    public static double armForwardPos = 0.2597;
    public static double armSidewayPos = 0.3148;
    public static double armBackwardPos = 0.369;

    // Side cones
    public static int countCones = 5; // number of side cones (changes value)
    public static double distancePerCone = 0.7; // distance in lift inches per side cone
    public static int sideConeLift = (int) (Constants.liftTicks * (3 + countCones * distancePerCone)); // calculation for lift height


    // Claw booleans
    public static boolean lastA = false;
    public static boolean direction = true;

    // Lift heights
    public static double lowLift = 7;
    public static double mediumLift = 13;
    public static double highLift = 19;
    public static double liftSpeed = 0.8;

    // Apriltag stuff
    public static AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public static AprilTagDetection tagOfInterest = null;
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;
    public static double tagsize = 0.166;

    // Vision choice
    public static int LEFT = 1;
    public static int MIDDLE = 2;
    public static int RIGHT = 3;
    public static int location = 0;

    // Roadrunner coordinates
    public static int startPoseX = 34;
    public static int startPoseY = -61;
    public static int depositX = 24;
    public static int depositY = -12;
    public static int loadX = 54;
    public static int loadY = -12;
    public static int parkX = 34 + 24 *  (location - 1);
    public static int parkY = depositY;
}
