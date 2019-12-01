package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import java.util.Arrays;

@TeleOp
public class PIDTuner extends OpMode {

    GroverHardware robot = new GroverHardware();

    ButtonToggle toggleX = new ButtonToggle();
    ButtonToggle toggleLeftStick = new ButtonToggle();

    ElapsedTime time = new ElapsedTime();
    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        if(gamepad1.a){
            double target = (int)robot.getHeading() + 90;
            while((int)robot.getHeading() != target && !gamepad1.b) {
                double c = robot.dt.gyroTurnCorrection(robot.getHeading(), target, robot.dt.turnToAnglePID);
                robot.dt.setMotorPower(-c, c, -c, c);
            }
            robot.dt.setMotorPower(0,0,0,0);

        }
        if (gamepad1.x) {
            robot.resetHeading();
        }
        if(gamepad1.dpad_up){
            if(gamepad1.left_trigger > 0.5){
                robot.dt.turnToAnglePID.setKp(robot.dt.turnToAnglePID.getKp() + 0.001);
            }
            else {
                robot.dt.turnToAnglePID.setKp(robot.dt.turnToAnglePID.getKp() + 0.0001);
            }
        }

        else if(gamepad1.dpad_down){
            if(gamepad1.left_trigger > 0.5){
                robot.dt.turnToAnglePID.setKp(robot.dt.turnToAnglePID.getKp() - 0.01);
            }
            else {
                robot.dt.turnToAnglePID.setKp(robot.dt.turnToAnglePID.getKp() - 0.001);
            }
        }
        telemetry.addData("Kp: ", robot.dt.turnToAnglePID.getKp());
        telemetry.addData("Ki: ", robot.dt.turnToAnglePID.getKi());
        telemetry.addData("Kd: ", robot.dt.turnToAnglePID.getKd());
        telemetry.addData("Angle: ", robot.getHeading());

    }

}
