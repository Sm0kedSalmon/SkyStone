package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp(name = "Grover TeleOp")
public class GroverTeleOp extends OpMode {

    //make an instance of the hardware class
    GroverHardware robot = new GroverHardware();

    //gamepad 1 toggles
    ButtonToggle toggleX = new ButtonToggle();

    //gamepad 2 toggles
    ButtonToggle toggleX2 = new ButtonToggle();
    ButtonToggle toggleDpadUp = new ButtonToggle();
    ButtonToggle toggleDpadDown = new ButtonToggle();
    ButtonToggle toggleB = new ButtonToggle();
    ButtonToggle toggleB2 = new ButtonToggle();
    ButtonToggle toggleLeftBumper = new ButtonToggle();
    ButtonToggle toggleBack = new ButtonToggle();
    ButtonToggle toggleDpadLeft = new ButtonToggle();
    //set up ftc dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    //the robot starts sideways from auto, so we need to make up for this
    private double angleOffset = 90;

    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){
        //choose angle offset because auto finishes in different orientations depending on alliance
        if(gamepad1.x) angleOffset = 90;
        else if(gamepad1.y) angleOffset = 0;
        else if(gamepad1.b) angleOffset = -90;
        else if(gamepad1.a) angleOffset = 180;

        telemetry.addData("Angle offset", angleOffset);
        telemetry.update();
    }

    public void start(){}

    public void loop(){

        /**
         * CONTROLS:
         *
         * GAMEPAD 1
         * Left joystick: moving around
         * Right joystick: turning in place
         * Left bumper: slow mode
         * B: PID gyro turn to 90 degrees
         * X: Toggle field centric mode
         * Left stick button: reset heading
         *
         * GAMEPAD 2
         * Right bumper: move intake
         * Left bumper: reverse intake
         * X: Toggle foundation gripper
         *
         * Dpad up: lift stage up
         * Dpad down: lift stage down
         * A: move lift up manually
         * Y: move lift down manually
         *
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
            double gyroAngle = robot.imu.getHeading() + angleOffset;
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

        //if a wheel power is greater than 1, divides each wheel power by the highest one
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
        else robot.dt.setMotorPower(FrontLeftVal * 0.85, FrontRightVal * 0.85, BackLeftVal * 0.85, BackRightVal * 0.85);

        //reset angle
        if(gamepad1.left_stick_button)
            angleOffset = -robot.imu.getHeading();


        //GAMEPAD 2 CONTROLS

        //Intake controls
        if(gamepad2.right_trigger > 0.5) robot.intake.on();
        else if(gamepad2.left_trigger > 0.5) robot.intake.reverse();
        else robot.intake.off();

        //Foundation gripper controls
        if(toggleX2.getState(gamepad2.x)) robot.foundationGripper.grab();
        else robot.foundationGripper.reset();

        //Lift controls
        //changing the set position
        if(toggleDpadUp.buttonPressed(gamepad2.dpad_up)) robot.lift.stageUp();
        else if(toggleDpadDown.buttonPressed(gamepad2.dpad_down)) robot.lift.stageDown();

        //toggle position correction on/off; it's on by default
        if(toggleLeftBumper.getState(gamepad2.left_bumper)) robot.lift.positionCorrection();
        else{
            //manual controls
            if (gamepad2.y && robot.lift.getMotorPosition() < robot.lift.MAX_HEIGHT) robot.lift.up();
            else if (gamepad2.a && robot.lift.limitSwitch.getState()) robot.lift.down();
            else robot.lift.stop();
        }

        //reset lift encoders
        if(gamepad2.left_stick_button) robot.lift.resetEncoder();

        //grabber arm
        if(toggleB2.getState(gamepad2.b)){
            robot.lift.moveOutsideRobot();
            telemetry.addLine("Arm Outside Robot");
        }
        else{
            if(toggleDpadLeft.getState(gamepad2.dpad_left))
                robot.lift.moveToCapstone();
            else
                robot.lift.moveInsideRobot();
        }


        //grab skystone
        if(toggleBack.getState(gamepad2.back))
            robot.lift.grabSkystone();
        else robot.lift.releaseSkystone();

        if(!robot.lift.limitSwitch.getState()){
            robot.lift.resetEncoder();
        }

        //Send data to the driver station
        telemetry.addData("Angle: ", robot.imu.getHeading());
        telemetry.update();

        packet.put("Heading", robot.imu.getHeading());

        dashboard.sendTelemetryPacket(packet);
    }

}