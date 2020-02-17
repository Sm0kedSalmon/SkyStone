package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.motionprofiling.MotionProfileGenerator;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

import dashboard.RobotConstants;

public class HydrofluoricLinearOpMode extends LinearOpMode {
    public GroverHardware robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    ElapsedTime time = new ElapsedTime();

    public HydrofluoricLinearOpMode(){

    }

    public void initHardware(GroverHardware robot){
        this.robot = robot;
    }

    public void runOpMode(){

    }

    //Autonomous driving, forwards or backwards.
    public void driveToPosition(double inches, double power){
        int TICKS = (int)(inches * robot.dt.TICKS_PER_INCH);

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition(robot.dt.FLMotor.getCurrentPosition() + TICKS);
        robot.dt.FRMotor.setTargetPosition(robot.dt.FRMotor.getCurrentPosition() + TICKS);
        robot.dt.BLMotor.setTargetPosition(robot.dt.BLMotor.getCurrentPosition() + TICKS);
        robot.dt.BRMotor.setTargetPosition(robot.dt.BRMotor.getCurrentPosition() + TICKS);

        robot.dt.runToPosition();

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(power,power,power,power);

        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy() && robot.dt.BRMotor.isBusy()){
            //correction method
        }

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
    }

    public void strafeToPosition(double inches, double power){
        int TICKS = (int)(inches * robot.dt.TICKS_PER_INCH);

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition(robot.dt.FLMotor.getCurrentPosition() + TICKS);
        robot.dt.FRMotor.setTargetPosition(robot.dt.FRMotor.getCurrentPosition() - TICKS);
        robot.dt.BLMotor.setTargetPosition(robot.dt.BLMotor.getCurrentPosition() - TICKS);
        robot.dt.BRMotor.setTargetPosition(robot.dt.BRMotor.getCurrentPosition() + TICKS);

        robot.dt.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dt.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dt.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dt.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(power,-power,-power,power);

        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy() && robot.dt.BRMotor.isBusy()){}

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
    }

    public void diagonalDriveNE(double inches, double power){
        int TICKS = (int)(inches * robot.dt.TICKS_PER_INCH);

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition(robot.dt.FLMotor.getCurrentPosition() + TICKS);
        robot.dt.BRMotor.setTargetPosition(robot.dt.BRMotor.getCurrentPosition() + TICKS);

        robot.dt.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dt.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(power,0,0,power);

        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.BRMotor.isBusy()){}

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
    }

    public void diagonalDriveNW(double inches, double power){
        int TICKS = (int)(inches * robot.dt.TICKS_PER_INCH);

        //Sets the target position.

        robot.dt.FRMotor.setTargetPosition(robot.dt.FRMotor.getCurrentPosition() + TICKS);
        robot.dt.BLMotor.setTargetPosition(robot.dt.BLMotor.getCurrentPosition() + TICKS);

        robot.dt.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.dt.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(0,power,power,0);

        while(opModeIsActive() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy()){}

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
    }

    public void encoderTurn(double angle, double power){

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition((int)(robot.dt.FLMotor.getCurrentPosition() - angle * robot.dt.TICKS_PER_DEGREE));
        robot.dt.FRMotor.setTargetPosition((int)(robot.dt.FRMotor.getCurrentPosition() + angle * robot.dt.TICKS_PER_DEGREE));
        robot.dt.BLMotor.setTargetPosition((int)(robot.dt.BLMotor.getCurrentPosition() - angle * robot.dt.TICKS_PER_DEGREE));
        robot.dt.BRMotor.setTargetPosition((int)(robot.dt.BRMotor.getCurrentPosition() + angle * robot.dt.TICKS_PER_DEGREE));

        robot.dt.runToPosition();

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(-power,power,-power,power);

        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy() && robot.dt.BRMotor.isBusy()){}

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
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
    
    //correct heading while driving with REV robot.imu
    public void driveAndCorrectAngle(double inches, double power, double targetAngle){
        int TICKS = (int)(inches *  robot.dt.TICKS_PER_INCH);

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition(robot.dt.FLMotor.getCurrentPosition() + TICKS);
        robot.dt.FRMotor.setTargetPosition(robot.dt.FRMotor.getCurrentPosition() + TICKS);
        robot.dt.BLMotor.setTargetPosition(robot.dt.BLMotor.getCurrentPosition() + TICKS);
        robot.dt.BRMotor.setTargetPosition(robot.dt.BRMotor.getCurrentPosition() + TICKS);

        robot.dt.runToPosition();

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(power,power,power,power);

        //angle correction while the robot is moving
        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy() && robot.dt.BRMotor.isBusy()){
            //gets PID value
            double c = gyroPIDCorrection(robot.imu.getHeading(), targetAngle, robot.dt.turnToAnglePID);

            //if it's going backward, the correction is the opposite
            if(inches < 0) c = -c;

            double FrontLeftVal = power - c;
            double FrontRightVal = power + c;
            double BackLeftVal = power - c;
            double BackRightVal = power + c;

            //if the largest power is greater than 1, divides each by the largest power
            double[] powers = {FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal};
            powers = robot.dt.normalizePowers(powers);

            //sets new motor powers
            robot.dt.setMotorPower(powers);

            packet.put("Angle", robot.imu.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }

        robot.dt.setMotorPower(0,0,0,0);

        robot.dt.enableEncoders();
    }

    //We use magic to drive sideways
    public void strafeAndCorrectAngle(double inches, double power, double targetAngle){
        int TICKS = (int)(inches * robot.dt.TICKS_PER_INCH);

        //Sets the target position.
        robot.dt.FLMotor.setTargetPosition(robot.dt.FLMotor.getCurrentPosition() + TICKS);
        robot.dt.FRMotor.setTargetPosition(robot.dt.FRMotor.getCurrentPosition() - TICKS);
        robot.dt.BLMotor.setTargetPosition(robot.dt.BLMotor.getCurrentPosition() - TICKS);
        robot.dt.BRMotor.setTargetPosition(robot.dt.BRMotor.getCurrentPosition() + TICKS);

        robot.dt.runToPosition();

        //Sets the motors to a certain power until the target position is reached.
        robot.dt.setMotorPower(power,-power,-power,power);

        //angle correction while the robot is moving
        while(opModeIsActive() && robot.dt.FLMotor.isBusy() && robot.dt.FRMotor.isBusy() && robot.dt.BLMotor.isBusy() && robot.dt.BRMotor.isBusy()){
            //gets PID value
            double c = gyroPIDCorrection(robot.imu.getHeading(), targetAngle, robot.dt.turnToAnglePID);

            //if it's going backward, the correction is the opposite
            if(inches < 0) c = -c;

            double FrontLeftVal = power - c;
            double FrontRightVal = power + c;
            double BackLeftVal = power - c;
            double BackRightVal = power + c;

            //if the largest power is greater than 1, divides each by the largest power
            double[] powers = {FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal};
            powers = robot.dt.normalizePowers(powers);

            //sets new motor powers
            robot.dt.setMotorPower(powers);

            robot.lift.positionCorrection();
        }

        robot.dt.setMotorPower(0,0,0,0);

        //strafing + moving lift at the same time
        robot.dt.enableEncoders();
    }

    public void gyroTurn(int degrees, double power){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        if(robot.imu.getHeading() < degrees && opModeIsActive()){
            //Turns until the angle is within a certain threshold
            while(opModeIsActive() && (Math.abs(robot.imu.getHeading()) < Math.abs(degrees) && time.seconds() < 3)) {
                robot.dt.setMotorPower(-power,power,-power,power);
            }
        }
        else if(robot.imu.getHeading() > degrees && opModeIsActive()){
            //Turns until the angle is within a certain threshold
            while(opModeIsActive() && (Math.abs(robot.imu.getHeading()) < Math.abs(degrees) && time.seconds() < 3)) {
                robot.dt.setMotorPower(power,-power,power,-power);
            }
        }
    }

    public void gyroTurnPID(int degrees){
        robot.dt.turnToAnglePID.reset();
        time.reset();

        //Turns until the angle is within a certain threshold
        while(opModeIsActive() && (Math.abs(robot.imu.getHeading() - degrees) > 0.1) && time.seconds() < 3) {
            //Gets an output value from the PID controller
            double c = gyroPIDCorrection(robot.imu.getHeading(), degrees, robot.dt.turnToAnglePID);
            robot.dt.setMotorPower(-c, c, -c, c);

            packet.put("Angle", robot.imu.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }

        robot.dt.resetEncoders();
    }

    public void straightMotionProfile(double inches, int angle){
        timeMotionProfile(inches, angle, "STRAIGHT");
    }
    public void sidewaysMotionProfile(double inches, int angle){
        timeMotionProfile(inches, angle, "SIDEWAYS");
    }
    public void diagonalMotionProfileNE(double inches, int angle){
        timeMotionProfile(inches, angle, "DIAGONAL_NE");
    }
    public void diagonalMotionProfileNW(double inches, int angle){
        timeMotionProfile(inches, angle, "DIAGONAL_NW");
    }

    public void timeMotionProfile(double inches, int angle, String direction){
        MotionProfileGenerator g = new MotionProfileGenerator(Math.abs(inches/2.0));

        double[] motionProfile = g.generateMotionProfile();
        double[] positionProfile = g.generatePositionProfile();

        robot.dt.motionProfilePID.reset();

        robot.dt.resetEncoders();
        ElapsedTime timeMillis = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timeMillis.reset();

        while(timeMillis.milliseconds() < motionProfile.length && opModeIsActive()){
            double velocity = motionProfile[(int)timeMillis.milliseconds()];
            double power = velocity / g.MAX_VELOCITY;

            double expectedPosition = positionProfile[(int)timeMillis.milliseconds()] * 2;
            double currentPosition = robot.dt.getAveragePosition() / robot.dt.TICKS_PER_INCH;

            robot.dt.motionProfilePID.setTarget(expectedPosition);
            robot.dt.motionProfilePID.updatePID(currentPosition);


            double positionFeedback = robot.dt.motionProfilePID.getOutput();

            if(inches < 0){
                power = -power;
                power -= positionFeedback;
            }
            else power += positionFeedback;


            /*double headingFeedback = gyroPIDCorrection(robot.imu.getHeading(), angle, robot.dt.turnToAnglePID);
            if(inches < 0) headingFeedback = -headingFeedback;

            double[] powers = {power - headingFeedback, power + headingFeedback, power - headingFeedback, power + headingFeedback};
            powers = robot.dt.normalizePowers(powers);*/

            if(direction.equals("STRAIGHT"))
                robot.dt.setMotorPower(power,power,power,power);
            else if(direction.equals("SIDEWAYS"))
                robot.dt.setMotorPower(power,-power,-power,power);
            else if(direction.equals("DIAGONAL_NE"))
                robot.dt.setMotorPower(power,0,0,power);
            else if(direction.equals("DIAGONAL_NW"))
                robot.dt.setMotorPower(0,power,power,0);

            //used to move lift while driving
            robot.lift.positionCorrection();

            packet.put("Power", power);
            packet.put("Position", robot.dt.getAveragePosition() / robot.dt.TICKS_PER_INCH);
            packet.put("Target Position", expectedPosition);
            dashboard.sendTelemetryPacket(packet);
        }



    }


}
