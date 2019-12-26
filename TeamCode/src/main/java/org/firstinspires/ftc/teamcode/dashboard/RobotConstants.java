package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //PID tuning values
    public static double GYRO_TURN_KP = 0.015;
    public static double GYRO_TURN_KI = 0.0007;
    public static double GYRO_TURN_KD = 0.0035;

    public static double TARGET_ANGLE = 90;

    public static double THRESHOLD = 0.002;

    //foundation auto values
    /*public static double FOUNDATION_1_DIAGONAL = 30.75;
    public static double FOUNDATION_2_STRAFE = -30.75;
    public static double DRIVE_LEFT_DISTANCE = 40;
    public static double ALIGN_DISTANCE = 12;*/

    public static double FIRST_TURN_DIST = 85;
    public static double SECOND_TURN_DIST = 85;

    public static double OUTTAKE_TIME = 2000;
    //lift and CV values, will be set later
    public static int LIFT_FIRST_STAGE = 0;
    public static int LIFT_SECOND_STAGE = 0;
    public static int LIFT_THIRD_STAGE = 0;
    public static int LIFT_FOURTH_STAGE = 0;
    public static int LIFT_FIFTH_STAGE = 0;
    /*public static int OFFSET_X;
    public static int OFFSET_Y;*/

    public static double ACCELERATION_Kp = 0.001;
    public static double MIN_POWER = 0.1;

    public static double TEST_POWER = 0.5;

    public static double LOADING_POSITION = 0.81;
}