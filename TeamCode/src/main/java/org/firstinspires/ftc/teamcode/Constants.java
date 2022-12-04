package org.firstinspires.ftc.teamcode;

public class Constants {
    public static double wheelDiameter = 3.77953;
    public static double circumference = wheelDiameter * Math.PI * 0.513;
    public static double circumferenceLift = 2 * Math.PI;
    public static double ticksPerRevolution = 537.6;
    public static int liftTicks = (int) (250 / circumferenceLift);  // variable value because of inconsistencies
    public static double turnDenom = 0.8;
    public static double denominator = 0.8;
    public static double openClaw = 0;
    public static double closedClaw = 1;
    public static boolean armInUse = false;
    public static double armForwardPos = 0.2597;
    public static double armSidewayPos = 0.3148;
    public static double armBackwardPos = 0.369;
    int signalZonePos = 0;
    boolean lastA = false;
    boolean direction = true;
    double lowLift = 7;
    double mediumLift = 13;
    double highLift = 19;

    public Constants() {

    }
}
