package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp
public class GroverTeleOp extends OpMode {

    GroverHardware robot = new GroverHardware();

    ButtonToggle toggleX = new ButtonToggle();

    public void init(){
        robot.init(hardwareMap);
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
         *
         **/

        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if(toggleX.getState(gamepad1.x)){
            double gyroAngle = robot.getHeading() * (Math.PI / 180);
            double joystickAngle = Math.atan2(y, x);

            double combinedAngle = joystickAngle - gyroAngle;

            x = Math.cos(combinedAngle);
            y = Math.sin(combinedAngle);
        }

        double FrontLeftVal = y + x + turn;
        double FrontRightVal = y - x - turn;
        double BackLeftVal = y - x + turn;
        double BackRightVal = y + x - turn;

        double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
        Arrays.sort(wheelPowers);
        if(wheelPowers[3] > 1){
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
        }

        if(gamepad1.right_trigger > 0.5) robot.dt.setMotorPower(FrontLeftVal/8,FrontRightVal/8,BackLeftVal/8,BackRightVal/8);
        else robot.dt.setMotorPower(FrontLeftVal,FrontRightVal,BackLeftVal,BackRightVal);

        //Intake controls
        if(gamepad1.right_bumper) robot.intake.intakeOn();
        else if(gamepad1.left_bumper) robot.intake.intakeReverse();
        else robot.intake.intakeOff();

        if(gamepad1.a){
            double c = robot.dt.gyroTurnCorrection(robot.getHeading(), 90);
            robot.dt.setMotorPower(-c,c,-c,c);
            telemetry.addData("P: ", c);
        }
    }

}