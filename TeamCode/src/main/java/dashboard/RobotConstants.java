package dashboard;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    //PID tuning values

    public static double GYRO_TURN_KP = 0.03;
    public static double GYRO_TURN_KI = 0;
    public static double GYRO_TURN_KD = 0.003;
    public static double GYRO_TURN_KIMAX = 0.5;


    public static double MOTION_PROFILE_Kp = 0.1;
    public static double MOTION_PROFILE_Ki = 0;
    public static double MOTION_PROFILE_Kd = 0;
    public static double MOTION_PROFILE_KiMAX = 0.5;

    public static double TARGET_ANGLE = 90;

    public static double GYRO_TURN_THRESHOLD = 0.002;

    public static double FIRST_TURN_DIST = 85;
    public static double SECOND_TURN_DIST = 85;

    public static double OUTTAKE_TIME = 2000;

    public static double MAX_VELOCITY = 27.5;
    public static double MAX_ACCELERATION = 20;

    public static double DISTANCE = 20;

    public static double FRONT_GRIPPER_GRAB_POS = 0.9;
    public static double BACK_GRIPPER_GRAB_POS = 0.45;

    public static double BACK_UP_DISTANCE = -22;

    public static double F_ALIGN_SIDEWAYS_DIST = -16;
    public static double F_DRIVE_INTO_FOUND_DIST = 30;
    public static double F_BACK_UP_DIST = -70;
    public static double F_SLAM_DIST = 4;
    public static double ALIGN_WITH_WALL_DIST = -30;
    public static double F_PARK_DIST = -40;

    public static double CAPSTONE_POSITION = 0.95;

    public static double SB_STRAFE_OFF_WALL = 5;
    public static double SB_ALIGN_WITH_QUARRY = 37;
    public static double SB_INTAKE_FIRST_STONE = 5; //15
    public static double SB_ALIGN_WITH_SKYBRIDGE = -8;
    public static double SB_ALIGN_WITH_FOUNDATION = 80;
    public static double SB_MOVE_AGAINST_FOUNDATION = 10;
    public static double SB_DRAG_FOUNDATION = -50;
    public static double SB_ALIGN_WITH_SKYBRIDGE_2 = 18;
    public static double SB_ZOOM_TO_SECOND_SKYSTONE = 60;
    public static double SB_ALIGN_WITH_SECOND_SKYSTONE = 10;
    public static double SB_PICK_UP_SECOND_STONE = 10;
    public static double SB_ALIGN_WITH_SKYBRIDGE_FINAL = 15;
    public static double SB_ZOOM_UNDER_SKYBRIDGE_FINAL;
    public static double SB_PARK;
}