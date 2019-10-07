package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
@TeleOp
public class FieldCentricTest extends OpMode {
    private static double JOYSTICK_DEADZONE = 0.1;

    GroverHardware robot = new GroverHardware();

    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        //moving
        double leftstickX = gamepad1.left_stick_x;
        double leftstickY = gamepad1.left_stick_y;

        //rotating
        double turn = gamepad1.right_stick_x;

        //gets the robot heading and converts it to radians
        double gyroAngle = robot.getHeading();
        gyroAngle *= (Math.PI / 180);

        //gets the distance
        double r = Math.hypot(leftstickX, leftstickY);
        double robotAngle = Math.atan2(leftstickY, leftstickX) - gyroAngle;

        double x = Math.cos(robotAngle);
        double y = Math.sin(robotAngle);

        double FLPower = r * (y + turn + x);
        double FRPower = r * (y - turn - x);
        double BLPower = r * (y + turn - x);
        double BRPower = r * (y - turn + x);

        //See if there are any powers above 1 or below -1
        double[] motorPowers = {FLPower, FRPower, BLPower, BRPower};

        double maxPow = 0;
        for(int i = 0; i < motorPowers.length; i++){
            if(i == 0 || Math.abs(motorPowers[i]) > maxPow)
                maxPow = Math.abs(motorPowers[i]);
        }

        //If so, divides each power by the largest power
        if(maxPow > 1){
            FLPower /= maxPow;
            FRPower /= maxPow;
            BLPower /= maxPow;
            BRPower /= maxPow;
        }

        //Only moves the robot if they joystick is moved a certain amount
        if(r >= JOYSTICK_DEADZONE)
            robot.dt.setMotorPower(FLPower,FRPower,BLPower,BRPower);
        else
            robot.dt.setMotorPower(0,0,0,0);

        updateTelemetry();
    }

    public void updateTelemetry(){
        telemetry.addData("Robot heading: ", robot.getHeading());
    }
}
