package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    public static final double TICKS_PER_MOTOR_REV = 1120; // Will change depending on encoder
    public static final double WHEEL_DIAMETER_INCHES = 3.54331; // Will change depending on wheel
    public static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;

    public Drivetrain(HardwareMap ahwMap){
        FLMotor  = ahwMap.get(DcMotor.class, "FLMotor");
        FRMotor = ahwMap.get(DcMotor.class, "FRMotor");
        BLMotor  = ahwMap.get(DcMotor.class, "BLMotor");
        BRMotor = ahwMap.get(DcMotor.class, "BRMotor");

        setMotorPower(0,0,0,0);

        //find out which direction each motor should be set to (forward or reverse?)

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //brake or no brake?
    }

    public void setMotorPower(float fl, float fr, float bl, float br){
        FLMotor.setPower(fl);
        FRMotor.setPower(fr);
        BLMotor.setPower(bl);
        BRMotor.setPower(br);
    }

    public void driveStraight(float inches, float power){
        int TICKS = (int)(inches * TICKS_PER_INCH);

        //Sets the target position.
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + TICKS);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + TICKS);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + TICKS);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + TICKS);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        setMotorPower(power,power,power,power);

        while(FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){}

        setMotorPower(0,0,0,0);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
