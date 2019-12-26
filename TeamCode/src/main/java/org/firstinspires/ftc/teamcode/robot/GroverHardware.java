package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;

import java.util.Arrays;

//Class that has all of the hardware of the robot, as well as some misc. methods.
public class GroverHardware {

    HardwareMap hwMap =  null;

    public Drivetrain dt = null;
    public Intake intake = null;
    public Lift lift = null;
    public FoundationGripper foundationGripper = null;

    public BNO055IMU imu;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

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
        foundationGripper = new FoundationGripper(hwMap);
    }

    //Returns the robot heading in degrees
    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void gyroTurnPID(int degrees){
        dt.turnToAnglePID.reset();
        time.reset();

        //Turns until the angle is within a certain threshold
        while((Math.abs(getHeading() - degrees) > 0.5) && time.seconds() < 5) {
            //Gets an output value from the PID controller
            double c = dt.gyroPIDCorrection(getHeading(), degrees, dt.turnToAnglePID);
            dt.setMotorPower(-c, c, -c, c);

            packet.put("Error", dt.turnToAnglePID.getTarget() - dt.turnToAnglePID.getCurrent());
            dashboard.sendTelemetryPacket(packet);
        }

        dt.setMotorPower(0,0,0,0);
        dt.resetEncoders();
    }

    //correct heading while driving with REV IMU
    public void driveAndCorrectAngle(double inches, double power, double targetAngle){
        int TICKS = (int)(inches * dt.TICKS_PER_INCH);

        //Sets the target position.
        dt.FLMotor.setTargetPosition(dt.FLMotor.getCurrentPosition() + TICKS);
        dt.FRMotor.setTargetPosition(dt.FRMotor.getCurrentPosition() + TICKS);
        dt.BLMotor.setTargetPosition(dt.BLMotor.getCurrentPosition() + TICKS);
        dt.BRMotor.setTargetPosition(dt.BRMotor.getCurrentPosition() + TICKS);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        dt.setMotorPower(power,power,power,power);

        //angle correction while the robot is moving
        while(dt.FLMotor.isBusy() && dt.FRMotor.isBusy() && dt.BLMotor.isBusy() && dt.BRMotor.isBusy()){
            //gets PID value
            double c = dt.gyroPIDCorrection(getHeading(), targetAngle, dt.turnToAnglePID);

            //if it's going backward, the correction is the opposite
            if(inches < 0) c = -c;

            double FrontLeftVal = power - c;
            double FrontRightVal = power + c;
            double BackLeftVal = power - c;
            double BackRightVal = power + c;

            //if the largest power is greater than 1, divides each by the largest power
            double[] powers = {FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal};
            Arrays.sort(powers);
            if(powers[3] > 1){
                FrontLeftVal /= powers[3];
                FrontRightVal /= powers[3];
                BackLeftVal /= powers[3];
                BackRightVal /= powers[3];
            }

            //sets new motor powers
            dt.setMotorPower(FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal);
        }

        dt.setMotorPower(0,0,0,0);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //We use magic to drive sideways
    public void strafeAndCorrectAngle(double inches, double power, double targetAngle){
        int TICKS = (int)(inches * dt.TICKS_PER_INCH);

        //Sets the target position.
        dt.FLMotor.setTargetPosition(dt.FLMotor.getCurrentPosition() + TICKS);
        dt.FRMotor.setTargetPosition(dt.FRMotor.getCurrentPosition() - TICKS);
        dt.BLMotor.setTargetPosition(dt.BLMotor.getCurrentPosition() - TICKS);
        dt.BRMotor.setTargetPosition(dt.BRMotor.getCurrentPosition() + TICKS);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        dt.setMotorPower(power,-power,-power,power);

        //angle correction while the robot is moving
        while(dt.FLMotor.isBusy() && dt.FRMotor.isBusy() && dt.BLMotor.isBusy() && dt.BRMotor.isBusy()){
            //gets PID value
            double c = dt.gyroPIDCorrection(getHeading(), targetAngle, dt.turnToAnglePID);

            //if it's going backward, the correction is the opposite
            if(inches < 0) c = -c;

            double FrontLeftVal = power - c;
            double FrontRightVal = power + c;
            double BackLeftVal = power - c;
            double BackRightVal = power + c;

            //if the largest power is greater than 1, divides each by the largest power
            double[] powers = {FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal};
            Arrays.sort(powers);
            if(powers[3] > 1){
                FrontLeftVal /= powers[3];
                FrontRightVal /= powers[3];
                BackLeftVal /= powers[3];
                BackRightVal /= powers[3];
            }

            //sets new motor powers
            dt.setMotorPower(FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal);
        }

        dt.setMotorPower(0,0,0,0);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //I wanted acceleration and deceleration to work but it doesn't and I'm sad :(
    public void driveAndCorrectAngleMotionProfile(double inches, double basePower, double targetAngle){
        int TICKS = (int)(inches * dt.TICKS_PER_INCH);
        double avgDistance = 0;

        dt.resetEncoders();
        while(Math.abs(avgDistance) < Math.abs(TICKS)){
            //ACCELERATION
            double Kp = RobotConstants.ACCELERATION_Kp;
            double power = basePower;

            avgDistance = Math.abs((dt.FLMotor.getCurrentPosition() + dt.FRMotor.getCurrentPosition()
                    + dt.BLMotor.getCurrentPosition() + dt.BRMotor.getCurrentPosition()) / 4);

            double error = Math.abs(TICKS - avgDistance);

            //slowing down
            if (error < Math.abs(basePower) / Kp) {
                power = Math.max(error * Kp, RobotConstants.MIN_POWER);
            }
            //speeding up
            else if (avgDistance < Math.abs(basePower) / Kp) {
                power = Math.max(avgDistance * Kp, RobotConstants.MIN_POWER);
            }

            if(TICKS < 0) power = -power;

            //HEADING CORRECTION
            //gets PID correction value
            double c = dt.gyroPIDCorrection(getHeading(), targetAngle, dt.turnToAnglePID);

            //if it's going backward, the correction is the opposite
            if(inches < 0) c = -c;

            double FrontLeftVal = power - c;
            double FrontRightVal = power + c;
            double BackLeftVal = power - c;
            double BackRightVal = power + c;

            //if the largest power is greater than 1, divides each by the largest power
            double[] powers = {FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal};
            Arrays.sort(powers);
            if(powers[3] > 1){
                FrontLeftVal /= powers[3];
                FrontRightVal /= powers[3];
                BackLeftVal /= powers[3];
                BackRightVal /= powers[3];
            }
            packet.put("Heading", getHeading());
            packet.put("Current power", power);
            packet.put("Distance", avgDistance);
            packet.put("Error", error);
            dashboard.sendTelemetryPacket(packet);

            //sets new motor powers
            dt.setMotorPower(FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal);
        }

        dt.setMotorPower(0,0,0,0);

        dt.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dt.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
