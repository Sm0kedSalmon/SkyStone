package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
@TeleOp
public class FieldCentricTest extends OpMode {
    private static double JOYSTICK_DEADZONE = 0.01;

    GroverHardware robot = new GroverHardware();

    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        double leftstickX = gamepad1.left_stick_x;
        double leftstickY = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        //gets the robot heading and converts it to radians
        double gyroAngle = robot.getHeading();
        gyroAngle *= (Math.PI / 180);

        //gets the distance
        double r = Math.hypot(leftstickX, leftstickY);
        double robotAngle = Math.atan2(leftstickY, leftstickX);

        //Holding left trigger --> field centric mode
        if(gamepad1.left_trigger > 0.5) robotAngle -= gyroAngle;

        double x = Math.cos(robotAngle);
        double y = Math.sin(robotAngle);

        double FLPower = r * (y + x);
        double FRPower = r * (y - x);
        double BLPower = r * (y - x);
        double BRPower = r * (y + x);

        //For now, we can't turn at the same time as driving
        if(r < JOYSTICK_DEADZONE) {
            FLPower += turn;
            FRPower -= turn;
            BLPower += turn;
            BRPower -= turn;
        }

        //See if there are any powers above 1 or below -1
        ArrayList<Double> motorPowers = new ArrayList<>(Arrays.asList(FLPower, FRPower, BLPower, BRPower));
        double maxPow = Collections.max(motorPowers);

        //If so, divides each power by the largest power
        if(maxPow > 1){
            FLPower /= maxPow;
            FRPower /= maxPow;
            BLPower /= maxPow;
            BRPower /= maxPow;
        }

        //Only moves the robot if they joystick is moved a certain amount
        if(r >= JOYSTICK_DEADZONE || Math.abs(gamepad1.right_stick_x) > JOYSTICK_DEADZONE){
            if(gamepad1.right_trigger >= 0.5)
                robot.dt.setMotorPower(FLPower/16,FRPower/16,BLPower/16,BRPower/16);
            else robot.dt.setMotorPower(FLPower/8,FRPower/8,BLPower/8,BRPower/8);
        }

        else robot.dt.setMotorPower(0,0,0,0);

        //PID turn to 90 degrees.
        if(gamepad1.a){
            double c = robot.dt.gyroTurnCorrection(robot.getHeading(), 90);
            robot.dt.setMotorPower(-c,c,-c,c);
            telemetry.addData("P: ", c);
        }

        //Intake controls
        if(gamepad1.y) robot.intake.intakeOn();
        else if(gamepad1.b) robot.intake.intakeReverse();
        else robot.intake.intakeOff();

        //Telemetry (shows text on the phone)
        telemetry.addData("Robot heading: ", robot.getHeading());
        telemetry.addData("Right joystick", gamepad1.right_stick_x);
        telemetry.addData("Turn", turn);
        telemetry.addData("FLPower", FLPower);


    }

}
