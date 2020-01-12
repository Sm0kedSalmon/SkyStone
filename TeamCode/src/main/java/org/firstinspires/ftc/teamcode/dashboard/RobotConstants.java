package org.firstinspires.ftc.teamcode.dashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //PID tuning values
    public static double GYRO_TURN_KP = 0.015;
    public static double GYRO_TURN_KI = 0.0007;
    public static double GYRO_TURN_KD = 0.0035;

    public static double MOTION_PROFILE_Kp = 0.1;
    public static double MOTION_PROFILE_Ki = 0;
    public static double MOTION_PROFILE_Kd = 0;

    public static double TARGET_ANGLE = 90;

    public static double THRESHOLD = 0.002;

    public static double FIRST_TURN_DIST = 85;
    public static double SECOND_TURN_DIST = 85;

    public static double OUTTAKE_TIME = 2000;

    public static double ACCELERATION_Kp = 0.001;
    public static double MIN_POWER = 0.1;

    public static double TEST_POWER = 0.5;


    public static double MAX_VELOCITY = 27.5;
    public static double MAX_ACCELERATION = 10;

    public static double DISTANCE = 20;



    public static double FRONT_GRIPPER_GRAB_POS = 1;
    public static double BACK_GRIPPER_GRAB_POS = 1;

    public static double OFFSET_X = 0;
    public static double OFFSET_Y = 0;

    public static double BACK_UP_DISTANCE = -17;

    public static double F_ALIGN_SIDEWAYS_DIST = -16;
    public static double F_DRIVE_INTO_FOUND_DIST = 30;
    public static double F_BACK_UP_DIST = -30;
    public static double F_SLAM_DIST = 4;
    public static double ALIGN_WITH_WALL_DIST = -30;
    public static double F_PARK_DIST = -52;

    public static double CAPSTONE_POSITION = 0.95;
}