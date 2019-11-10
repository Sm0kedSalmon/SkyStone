package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.misc.PID;

//All of the hardware + methods for the drivetrain
public class Drivetrain {

    public static final double TICKS_PER_MOTOR_REV = 1120; // Will change depending on encoder
    public static final double WHEEL_DIAMETER_INCHES = 3.93701; // Will change depending on wheel
    public static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public DcMotor FLMotor;
    public DcMotor FRMotor;
    public DcMotor BLMotor;
    public DcMotor BRMotor;

    //PID values need to be tuned
    public PID turnToAnglePID = new PID(0.0003,0,0);

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

    //Autonomous driving, forwards or backwards.
    public void driveStraight(double inches, double power){
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

        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveAtAngle(double xInches, double yInches, double power){
        int xTICKS = (int)(xInches * TICKS_PER_INCH);
        int yTICKS = (int)(yInches * TICKS_PER_INCH);

        //Sets the target position.
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + yTICKS + xTICKS);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + yTICKS - xTICKS);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + yTICKS - xTICKS);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + yTICKS + xTICKS);

        //converting x and y to the unit circle (normalize x and y)
        double angle = Math.atan(yInches / xInches);
        double nX = Math.cos(angle);
        double nY = Math.sin(angle);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setMotorPower((nY + nX) * power, (nY - nX) * power, (nY - nX * power), (nY + nX) * power);

        while(FLMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    //Represents 1 loop of a gyro turn PID loop. Called repeatedly until the target is reached.
    public double gyroTurnCorrection(double current, double target){
        double error = target - current;

        //if the error is greater than 180, we know it has to cross the 180 -180 boundary, so we switch to a 0-360 scale
        if(Math.abs(error) > 180){
            //normalize angles
            current = (current + 360) % 360;
            target = (target + 360) % 360;

            error = target - current;
        }

        //resets the target in case it's changed
        turnToAnglePID.setTarget(target);

        //
        turnToAnglePID.updatePID(current);

        //gets the output value, which is set in the method updatePID()
        return turnToAnglePID.getOutput();
    }


}
