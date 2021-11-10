package org.firstinspires.ftc.teamcode.hardware;

// constant variables for drive algorithms
public class ConstantVariables {

    public static final double WHEEL_DIAMETER = 3.77953; // diameter of wheels (this one is gobuilda mecanum)
    public static final int GEAR_RATIO = 20; // gear ratio of motors (this one is 20:1)
    public static final int COUNTS_PER_ROTATION = 37; // increase/decrease during calibration
    public static final double COUNTS_PER_INCH = (COUNTS_PER_ROTATION * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); //counts per inch, cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    public static final double BIAS = 1.0;

    // Get Vuforia key for free from https://developer.vuforia.com/license-manager
    public static final String VUFORIA_KEY = "AVq7Z/f/////AAABmTn41sddFULaqEWgfGwBTyKJSPmOp1LNUQbI0bedCPrPqodi7SxbjmrgL/Uph4zGgJykNsTlOCOhuX7JW+AGDep0cw/Tbro0J6If6p8AE5EV865roJ4+34h4swGhxH/dDvmVzAua2BECWQ9wAaYIT3cSw/VB1eHOvpSV1xwSAwE/S+n4iGEgL86Wt4c1ClFpUW/7CbYEVu0FXnnghmKAHI5jncXp8KQ1Ik3HYARoZsISI/Vbudhc+HE+pO5iKcV5cJElLIDmOSeKIwmbdiF4rzW9GbwHBcLVFO8T+1JyEnB3x/sCkRkH18Z9iEnDHTKT/v/X0FpyzRcYS06iD9p74CVgg1ne+cDKN4KAoeLYRwCy";

    // Below is old constant variables

    public static final int K_PPR_DRIVE = 1120;
    public static final double K_DRIVE_WHEEL_DIA = 3.77953;
    public static final double K_DRIVE_DIA = 16.5;//Dont know what this is.

    public static final double K_DRIVE_WHEEL_CIRCUMFERENCE = K_DRIVE_WHEEL_DIA * Math.PI; //12.56637
    public static final double K_PPIN_DRIVE = K_PPR_DRIVE / K_DRIVE_WHEEL_CIRCUMFERENCE; //89.1267725

    public static final double K_TURN_CIRCUMFERENCE = K_DRIVE_DIA * Math.PI;
    public static final double K_PPTURN_DRIVE = K_PPIN_DRIVE * K_TURN_CIRCUMFERENCE;
    public static final double K_PPDEG_DRIVE = K_PPTURN_DRIVE / 360;

    public static final double K_MAX_CLIBER = 1465;

    public static final double K_DRIVE_ERROR_P = 250; // higher = less sensitive

    public static final double K_ARM_ROTATE_PPR = 288; //encoder counts per rotation of arm rotator

    public static final double K_ARM_GEAR_RATIO = 1.0/2;
}
