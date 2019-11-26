package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.misc.PID;

public class Lift {
    public DcMotor liftMotor;

    public static double HOME_POSITION = 0;

    public static double BLOCK_HEIGHT = 0;

    public static double LIFT_POWER = 1;

    public static double MAX_HEIGHT = 5400;

    public static double[] positions = {0,1000,2000,3000,4000};

    private int currentPosition = 0;

    public Lift(HardwareMap ahwMap){
        liftMotor = ahwMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public double getPosition(){
        return liftMotor.getCurrentPosition();
    }
    public void resetEncoder(){
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void stageUp() {
        if(currentPosition < 5){
            currentPosition += 1;
        }
    }
    public void stageDown() {
        if(currentPosition > 0){
            currentPosition -= 1;
        }
    }
    public void positionCorrection(){
        double error = positions[currentPosition] - getPosition();

        liftMotor.setPower(error * 0.0005);
    }
}
