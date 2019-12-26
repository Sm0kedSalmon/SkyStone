package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp
public class GroverNoGyro extends OpMode {

    GroverHardware robot = new GroverHardware();

    ButtonToggle toggleDpadUp = new ButtonToggle();
    ButtonToggle toggleDpadDown = new ButtonToggle();

    ButtonToggle toggleX = new ButtonToggle();
    ButtonToggle toggleB = new ButtonToggle();

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
         * Y: move lift up
         * A: move lift down
         *
         * Dpad up: move lift to the set position above the current one
         * Dpad down: move lift to the set position below the current one
         *
         * Left stick button: reset lift
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
        if(gamepad1.right_trigger > 0.5) robot.intake.on();
        else if(gamepad1.left_trigger > 0.5) robot.intake.reverse();
        else robot.intake.off();

        //Lift controls

        //chaning the set position
        if(toggleDpadUp.buttonPressed(gamepad1.dpad_up)) robot.lift.stageUp();
        else if(toggleDpadDown.buttonPressed(gamepad1.dpad_down)) robot.lift.stageDown();

        //manual controls
        if (gamepad1.y && robot.lift.getMotorPosition() < robot.lift.MAX_HEIGHT)
            robot.lift.up();
        else if (gamepad1.a && robot.lift.getMotorPosition() > robot.lift.HOME_POSITION)
            robot.lift.down();

        //if the lift isn't being controlled manually, it automatically goes to the set position
        else robot.lift.positionCorrection();

        //reset lift encoders
        if(gamepad1.left_stick_button)
            robot.lift.resetEncoder();

        //skystone grabber
        if(toggleX.getState(gamepad1.x))
            robot.lift.grabSkystone();

        else
            robot.lift.releaseSkystone();

        //grabber arm
        if(toggleB.getState(gamepad1.b))
            robot.lift.moveOutsideRobot();
        else
            robot.lift.moveInsideRobot();

        telemetry.addData("Lift encoder position:", robot.lift.getMotorPosition());
        telemetry.addData("Lift target position:", robot.lift.getCurrentTargetPosition());

    }

}