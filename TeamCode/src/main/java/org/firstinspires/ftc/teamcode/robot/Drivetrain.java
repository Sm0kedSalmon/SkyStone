package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.misc.PID;

import java.util.Arrays;

//All of the hardware + methods for the drivetrain
public class Drivetrain {

    public static final double TICKS_PER_MOTOR_REV = 560; // Will change depending on encoder
    public static final double WHEEL_DIAMETER_INCHES = 3.93701; // Will change depending on wheel
    public static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double TICKS_PER_ROBOT_ROTATION = 3375;
    public static final double TICKS_PER_DEGREE = TICKS_PER_ROBOT_ROTATION / 360;

    public DcMotor FLMotor;
    public DcMotor FRMotor;
    public DcMotor BLMotor;
    public DcMotor BRMotor;

    //PID values need to be tuned
    //public PID turnToAnglePIDTest = new PID(RobotConstants.GYRO_TURN_KP,RobotConstants.GYRO_TURN_KI,RobotConstants.GYRO_TURN_KD);
    public PID turnToAnglePIDTest = new PID(0.0125,0.00001,0.0004);
    public PID turnToAnglePID = new PID(0.0125,0.00001,0.0004);
    //public PID turnToAngleN90 = new PID(0,0,0);
    public PID teleOpTurnToAnglePID = new PID(0.02,0,0.0015);
    //public PID motionProfilePID = new PID(RobotConstants.MOTION_PROFILE_Kp,RobotConstants.MOTION_PROFILE_Ki,RobotConstants.MOTION_PROFILE_Kd);
    public PID motionProfilePID = new PID(0.01,0,0);
    //Initializes motors
    public Drivetrain(HardwareMap ahwMap){
        FLMotor  = ahwMap.get(DcMotor.class, "FLMotor");
        FRMotor = ahwMap.get(DcMotor.class, "FRMotor");
        BLMotor  = ahwMap.get(DcMotor.class, "BLMotor");
        BRMotor = ahwMap.get(DcMotor.class, "BRMotor");

        setMotorPower(0,0,0,0);

        //find out which direction each motor should be set to (forward or reverse?)
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        //The motors will run without encoders on by default
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //When the robot stops, it will stop immediately
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorPower(double fl, double fr, double bl, double br){
        FLMotor.setPower(fl);
        FRMotor.setPower(fr);
        BLMotor.setPower(bl);
        BRMotor.setPower(br);
    }

    public void resetEncoders(){
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getAveragePosition(){
        return Math.abs((FLMotor.getCurrentPosition() + FRMotor.getCurrentPosition()
                + BLMotor.getCurrentPosition() + BRMotor.getCurrentPosition()) / 4);
    }

    public void disableEncoders(){
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Autonomous driving, forwards or backwards.
    public void driveToPosition(double inches, double power){
        int TICKS = (int)(inches * TICKS_PER_INCH);

        //Sets the target position.
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + TICKS);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + TICKS);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + TICKS);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + TICKS);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(power,power,power,power);

        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){
            //correction method
        }

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeToPosition(double inches, double power){
        int TICKS = (int)(inches * TICKS_PER_INCH);

        //Sets the target position.
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + TICKS);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() - TICKS);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() - TICKS);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + TICKS);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(power,-power,-power,power);

        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalDriveNE(double inches, double power){
        int TICKS = (int)(inches * TICKS_PER_INCH);

        //Sets the target position.
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + TICKS);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + TICKS);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(power,0,0,power);

        while(FLMotor.isBusy() && BRMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalDriveNW(double inches, double power){
        int TICKS = (int)(inches * TICKS_PER_INCH);

        //Sets the target position.

        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + TICKS);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + TICKS);

        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(0,power,power,0);

        while(FRMotor.isBusy() && BLMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderTurn(double angle, double power){

        //Sets the target position.
        FLMotor.setTargetPosition((int)(FLMotor.getCurrentPosition() - angle * TICKS_PER_DEGREE));
        FRMotor.setTargetPosition((int)(FRMotor.getCurrentPosition() + angle * TICKS_PER_DEGREE));
        BLMotor.setTargetPosition((int)(BLMotor.getCurrentPosition() - angle * TICKS_PER_DEGREE));
        BRMotor.setTargetPosition((int)(BRMotor.getCurrentPosition() + angle * TICKS_PER_DEGREE));

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(-power,power,-power,power);

        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Represents 1 loop of a gyro turn PID loop or correction PID loop. Called repeatedly until the target is reached.
    public double gyroPIDCorrection(double current, double target, PID pid){
        double error = target - current;

        //if the error is greater than 180, we know it has to cross the 180 -180 boundary, so we switch to a 0-360 scale
        //this is called an "angle wrap"
        if(Math.abs(error) > 180){
            current = (current + 360) % 360;
            target = (target + 360) % 360;
        }

        //resets the target in case it's changed
        pid.setTarget(target);

        //does calculations using PID controller
        pid.updatePID(current);

        //gets the output value, which is set in the method updatePID()
        return pid.getOutput();
    }



    public double[] normalizePowers(double[] powers){
        Arrays.sort(powers);
        if(powers[3] > 1){
            powers[0] /= powers[3];
            powers[1] /= powers[3];
            powers[2] /= powers[3];
            powers[3] /= powers[3];
        }
        return powers;
    }


}
