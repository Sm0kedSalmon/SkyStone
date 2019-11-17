package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp
public class GroverNoGyro extends OpMode {

    GroverHardware robot = new GroverHardware();

    ButtonToggle toggleX = new ButtonToggle();
    ButtonToggle toggleLeftStick = new ButtonToggle();

    public void init(){
        robot.initNoGyro(hardwareMap);
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

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double r = Math.hypot(x, y);

        double FrontLeftVal = r * (y + x) + turn;
        double FrontRightVal = r * (y - x) - turn;
        double BackLeftVal = r * (y - x) + turn;
        double BackRightVal = r * (y + x) - turn;

        double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
        Arrays.sort(wheelPowers);
        if(wheelPowers[3] > 1){
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
        }

        if (gamepad1.left_bumper)
            robot.dt.setMotorPower(FrontLeftVal / 4, FrontRightVal / 4, BackLeftVal / 4, BackRightVal / 4);
        else robot.dt.setMotorPower(FrontLeftVal / 2, FrontRightVal / 2, BackLeftVal / 2, BackRightVal / 2);

        //Intake controls
        if(gamepad1.right_trigger > 0.5) robot.intake.intakeOn();
        else if(gamepad1.left_trigger > 0.5) robot.intake.intakeReverse();
        else robot.intake.intakeOff();

    }

}