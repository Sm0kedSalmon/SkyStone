package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    ButtonToggle toggleX2 = new ButtonToggle();
    ButtonToggle toggleLeftStick = new ButtonToggle();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private double angleOffset = 90;

    private double currentTargetAngle;

    public void init(){
        robot.init(hardwareMap);
        currentTargetAngle = robot.getHeading();
    }

    public void init_loop(){
        if(gamepad1.x){
            angleOffset = 90;
        }
        else if(gamepad1.b) angleOffset = -90;

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
         * Right trigger: slow mode
         * X: Toggle field centric mode
         *
         * GAMEPAD 2
         * Right bumper: move intake
         * Left bumper: reverse intake
         * X: Toggle foundation gripper
         *
         */

        //gets the inputs from both joysticks
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //r is the total joystick displacement
        double r = Math.hypot(x, y);

        //if the robot is turning, it updates the current target angle.
        currentTargetAngle = turn > 0  || r < 0.01|| gamepad1.a ? robot.getHeading() : currentTargetAngle;



        //Pressing X switches from regular mode to field centric mode
        //If not in field centric mode, skips everything in the if method
        if(toggleX.getState(gamepad1.x)){
            //gets the robot heading and converts it to radians
            double gyroAngle = robot.getHeading() + angleOffset;
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

        //Holding the A button uses a PID loop to gradually turn to 90 degrees.
        if(gamepad1.a){
            double c = robot.dt.gyroPIDCorrection(robot.getHeading(), 90, robot.dt.teleOpTurnToAnglePID);
            robot.dt.setMotorPower(-c,c,-c,c);
            FrontLeftVal -= c;
            FrontRightVal += c;
            BackLeftVal -= c;
            BackRightVal += c;

            telemetry.addData("Motor output: ", c);
        }

        //Otherwise, corrects the heading. (tune this so it's more accurate)
        else if(r > 0.01){
            double c = robot.dt.gyroPIDCorrection(robot.getHeading(), currentTargetAngle, robot.dt.headingCorrectionPID);
            FrontLeftVal -= c;
            FrontRightVal += c;
            BackLeftVal -= c;
            BackRightVal += c;

            telemetry.addData("Correction power: ", c);
        }

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
        else robot.dt.setMotorPower(FrontLeftVal, FrontRightVal, BackLeftVal, BackRightVal);



        //Intake controls
        if(gamepad2.right_trigger > 0.5) robot.intake.on();
        else if(gamepad2.left_trigger > 0.5) robot.intake.reverse();
        else robot.intake.off();



        if(toggleX2.getState(gamepad2.x)){
            robot.gripper.grab();
        }
        else{
            robot.gripper.reset();
        }

        //reset angle
        if(toggleLeftStick.buttonPressed(gamepad1.left_stick_button)){
            angleOffset = robot.getHeading();
        }



        //Send data to the driver station
        telemetry.addData("Angle: ", robot.getHeading());
        telemetry.update();

        packet.put("Heading", robot.getHeading());
        packet.put("Target heading", currentTargetAngle);
        dashboard.sendTelemetryPacket(packet);
        dashboard.sendTelemetryPacket(packet);
    }

}