package org.firstinspires.ftc.teamcode;

public class Constants {

    /* public means it can be accessed by the other classes
    static means you can call the variables from the class (without reference object)
    final means the value is constant, can't be changed
    after that comes the object type
    then the name of the object, then the value after the equal sign
     */

    // claw positions
    public static final double openClaw = 0.27;
    public static final double closedClaw = 1;

    // arm positions
    public static final double armForwardPos = 0.31;
    public static final double arm145ANGLE = 0.55;
    public static final double armSidewayPos = 0.695;
    public static final double armPrevBackPos = 0.9;
    public static final double armBackwardPos = 0.975;

    // Side cones
    public static int countCones = 4; // number of side cones (changes value)

    // Claw booleans
    public static boolean lastA = false;
    public static boolean direction = true;

    // Lift heights

    public static final double factor = 13.03/18.88;
    public static final int lowLift = (int)(1106 * factor);
    public static final int mediumLift = (int)(1746 * factor);
    public static final int highLift = 1700;
    public static double liftSpeed = 0.8;

}