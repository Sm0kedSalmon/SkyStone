package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public DcMotor liftMotor;

    public static double HOME_POSITION = 0;
    public static double LIFT_POWER = 1;
    //public static double MAX_HEIGHT = RobotConstants.LIFT_MAX_POSITION;
    public static double MAX_HEIGHT = 5000;
    /*public static int[] positions = {0,
            RobotConstants.LIFT_FIRST_STAGE,
            RobotConstants.LIFT_SECOND_STAGE,
            RobotConstants.LIFT_THIRD_STAGE,
            RobotConstants.LIFT_FOURTH_STAGE,
            RobotConstants.LIFT_FIFTH_STAGE};*/
    public static int[] positions = {0,543,1783,2600,3317,4034};
    public int currentPosition = 0;


    public Servo skystoneGrabber;
    public static double GRAB_POSITION = 1;
    public static double RELEASE_POSITION = 0;

    public Servo rotator;
    public static double LOAD_POSITION = 0.85;
    public static double CAPSTONE_POSITION = 0.8;
    public static double STACK_POSITION = 0.12;
    public static double MIN_ROTATE_POSITION = 2500;

    public DigitalChannel limitSwitch;

    public Lift(HardwareMap ahwMap){
        liftMotor = ahwMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        skystoneGrabber = ahwMap.get(Servo.class, "skystone_grabber");
        skystoneGrabber.setPosition(RELEASE_POSITION);

        rotator = ahwMap.get(Servo.class, "rotating_arm");
        rotator.setPosition(LOAD_POSITION);

        limitSwitch = ahwMap.get(DigitalChannel.class, "lift_limit_switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void up(){
        liftMotor.setPower(LIFT_POWER);
    }
    public void down(){
        liftMotor.setPower(-LIFT_POWER);
    }
    public void stop(){
        liftMotor.setPower(0);
    }

    public double getMotorPosition(){
        return liftMotor.getCurrentPosition();
    }

    public void resetEncoder(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stageUp() {
        if(currentPosition < 4){
            currentPosition += 1;
        }
    }
    public void stageDown() {
        if(currentPosition > 0){
            currentPosition -= 1;
        }
    }

    public void positionCorrection(){
        double error = positions[currentPosition] - getMotorPosition();

        liftMotor.setPower(error * 0.001);
    }
    public double getCurrentTargetPosition(){
        return positions[currentPosition];
    }

    public void grabSkystone(){
        skystoneGrabber.setPosition(GRAB_POSITION);
    }
    public void releaseSkystone(){
        skystoneGrabber.setPosition(RELEASE_POSITION);
    }

    public void moveInsideRobot(){
        if(getMotorPosition() > MIN_ROTATE_POSITION)
            rotator.setPosition(LOAD_POSITION);
    }
    public void moveOutsideRobot(){
        if(getMotorPosition() > MIN_ROTATE_POSITION)
            rotator.setPosition(STACK_POSITION);
    }
    public void moveToCapstone(){
        rotator.setPosition(0.85);
    }

    public void movetoFoundationDeposit(){
        rotator.setPosition(1);
    }

    public void home(){
        /*if(rotator.getPosition() == STACK_POSITION){
            if(getMotorPosition() <= MIN_ROTATE_POSITION) liftMotor.setPower(1);
            else moveInsideRobot();
        }*/
        if(rotator.getPosition() == LOAD_POSITION && limitSwitch.getState()){
            liftMotor.setPower(-1);
        }
        if(!limitSwitch.getState()){
            currentPosition = 0;
            liftMotor.setPower(0);
        }

    }

    public void setCurrentPosition(int position){
        currentPosition = position;
    }

}
