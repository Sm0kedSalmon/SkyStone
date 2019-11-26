package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Class that has all of the hardware of the robot, as well as some misc. methods.
public class GroverHardware {

    HardwareMap hwMap =  null;

    public Drivetrain dt = null;
    public Intake intake = null;
    public Lift lift = null;

    public BNO055IMU imu;

    public GroverHardware(){

    }

    //Initializes hardware in each OpMode
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);

        //Set up IMU parameters
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }

    public void initNoGyro(HardwareMap ahwMap){
        hwMap = ahwMap;

        dt = new Drivetrain(hwMap);
        intake = new Intake(hwMap);
        lift = new Lift(hwMap);
    }

    //Returns the robot heading in degrees
    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void resetHeading(){

    }

    public void gyroTurnPID(int degrees){
        while(Math.abs(getHeading() - degrees) > 0.1) {
            double c = dt.gyroTurnCorrection(getHeading(), degrees, dt.autoTurnToAnglePID);
            dt.setMotorPower(-c, c, -c, c);
        }
        dt.setMotorPower(0,0,0,0);
    }

    public void encoderTurn(double angle, double power){
        double error = angle - getHeading();
        //Sets the target position.
        dt.FLMotor.setTargetPosition((int)(dt.FLMotor.getCurrentPosition() - error * dt.TICKS_PER_DEGREE));
        dt.FRMotor.setTargetPosition((int)(dt.FRMotor.getCurrentPosition() + error * dt.TICKS_PER_DEGREE));
        dt.BLMotor.setTargetPosition((int)(dt.BLMotor.getCurrentPosition() - error * dt.TICKS_PER_DEGREE));
        dt.BRMotor.setTargetPosition((int)(dt.BRMotor.getCurrentPosition() + error * dt.TICKS_PER_DEGREE));

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        dt.setMotorPower(-power,power,-power,power);

        while(dt.FLMotor.isBusy() && dt.FRMotor.isBusy() && dt.BLMotor.isBusy() && dt.BRMotor.isBusy()){}

        dt.setMotorPower(0,0,0,0);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
