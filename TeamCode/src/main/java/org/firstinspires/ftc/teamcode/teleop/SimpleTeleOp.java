package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
@TeleOp
public class SimpleTeleOp extends OpMode {

    GroverHardware robot = new GroverHardware();


    public void init(){
        robot.init(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    public void loop(){
        double FLPower = 0;
        double FRPower = 0;
        double BLPower = 0;
        double BRPower = 0;
        //moving
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        //rotating
        double turn = gamepad1.right_stick_x;

        FLPower = (y + x);
        FRPower = (y - x);
        BLPower = (y - x);
        BRPower = (y + x);

        FLPower += turn;
        FRPower -= turn;
        BLPower += turn;
        BRPower -= turn;

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


        robot.dt.setMotorPower(FLPower/4,FRPower/4,BLPower/4,BRPower/4);

        if(gamepad1.y) robot.intake.intakeOn();
        else if(gamepad1.b) robot.intake.intakeReverse();
        else robot.intake.intakeOff();

        telemetry.addData("Robot heading: ", robot.getHeading());
        telemetry.addData("Right joystick", gamepad1.right_stick_x);
        telemetry.addData("Turn", turn);
        telemetry.addData("FLPower", FLPower);


    }

}