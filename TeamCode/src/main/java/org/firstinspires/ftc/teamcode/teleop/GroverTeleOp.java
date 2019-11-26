package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp
public class GroverTeleOp extends OpMode {

    GroverHardware robot = new GroverHardware();

    ButtonToggle toggleX = new ButtonToggle();
    ButtonToggle toggleLeftStick = new ButtonToggle();

    public void init(){
        robot.init(hardwareMap);
        robot.dt.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dt.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){

        /**
         * CONTROLS:
         *
         * Left joystick: moving around
         * Right joystick: turning in place
         * Right trigger: slow mode
         *
         * Right bumper: move intake
         * Left bumper: reverse intake
         *
         * A: PID turn to 90 degrees
         * X: Toggle field centric mode
         */

        //gets the inputs from both joysticks
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //r is the total joystick displacement
        double r = Math.hypot(x, y);

        //Pressing X switches from regular mode to field centric mode
        if(toggleX.getState(gamepad1.x)){
            //gets the robot heading and converts it to radians
            double gyroAngle = robot.getHeading();
            gyroAngle *= (Math.PI / 180);

            double joystickAngle = Math.atan2(y, x);

            //subtracting the gyro angle from the joystick angle gets the movement angle relative to the driver
            double movementAngle = joystickAngle - gyroAngle;

            //converts the angle back into x and y
            x = Math.cos(movementAngle);
            y = Math.sin(movementAngle);

            telemetry.addLine("Field Centric ON");
        }

        //y and x represent the components of the vectors of the mecanum wheels.
        double FrontLeftVal = r * (y + x) + turn;
        double FrontRightVal = r * (y - x) - turn;
        double BackLeftVal = r * (y - x) + turn;
        double BackRightVal = r * (y + x) - turn;

        //if a wheel power is greater than 1, normalizes all of the wheel powers
        double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
        Arrays.sort(wheelPowers);
        if(wheelPowers[3] > 1){
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
        }

        //Left bumper activates quarter speed. Otherwise, goes at half speed.
        if (gamepad1.left_bumper)
            robot.dt.setMotorPower(FrontLeftVal / 4, FrontRightVal / 4, BackLeftVal / 4, BackRightVal / 4);
        else robot.dt.setMotorPower(FrontLeftVal / 2, FrontRightVal / 2, BackLeftVal / 2, BackRightVal / 2);

        //Intake controls
        if(gamepad1.right_trigger > 0.5) robot.intake.on();
        else if(gamepad1.left_trigger > 0.5) robot.intake.reverse();
        else robot.intake.off();

        //Holding the A button uses a PID loop to gradually turn to 90 degrees.
        if(gamepad1.a){
            double c = robot.dt.gyroTurnCorrection(robot.getHeading(), 90, robot.dt.turnToAnglePID);
            robot.dt.setMotorPower(-c,c,-c,c);
            telemetry.addData("Motor output: ", c);
        }

        //Send data to the driver station
        telemetry.addData("Angle: ", robot.getHeading());
        telemetry.addData("Left motor position", robot.dt.FLMotor.getCurrentPosition());
    }

}
