package org.firstinspires.ftc.teamcode;

public class Constants {

    /* public means it can be accessed by the other classes
    static means you can call the variables from the class (without reference object)
    final means the value is constant, can't be changed
    after that comes the object type
    then the name of the object, then the value after the equal sign
     */

    // claw positions
    public static final double openClaw = 0.2;
    public static final double closedClaw = 1;

    // arm positions
    public static double offset = 0.03;
    public static final double armForwardPos = 0.31;
    public static final double armSidewayPos = 0.635;
    public static final double armPrevBackPos = 0.7825;
    public static final double armBackwardPos = 0.93;
    public static final double armAutonMedPos = armBackwardPos - ((armBackwardPos - armSidewayPos)/2);

    // Side cones
    public static int countCones = 4; // number of side cones (changes value)

    // Claw booleans
    public static boolean lastA = false;
    public static boolean direction = true;

    // Lift heights
    public static final int lowLift = 495;
    public static final int mediumLift = 775;
    public static final int highLift = 1120;
    public static double liftSpeed = 0.8;

}