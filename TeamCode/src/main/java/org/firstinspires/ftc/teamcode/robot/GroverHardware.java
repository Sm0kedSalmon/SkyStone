package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.hardware.IMU;

//Class that has all of the hardware of the robot, as well as some misc. methods.
public class GroverHardware {

    HardwareMap hwMap =  null;

    public Drivetrain dt = null;
    public Intake intake = null;
    public Lift lift = null;
    public FoundationGripper foundationGripper = null;

    public IMU imu;


    public ElapsedTime time = new ElapsedTime();

    public GroverHardware(){

    }

    //Initializes hardware in each OpMode
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);
        foundationGripper = new FoundationGripper(hwMap);
        imu = new IMU(hwMap);
    }

    public void initNoGyro(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);
        foundationGripper = new FoundationGripper(hwMap);
    }

}
