package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.ButtonToggle;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
@TeleOp
public class LiftControlTest extends OpMode {
    GroverHardware robot = new GroverHardware();
    //gamepad 2 toggles
    ButtonToggle toggleX2 = new ButtonToggle();
    ButtonToggle toggleDpadUp = new ButtonToggle();
    ButtonToggle toggleDpadDown = new ButtonToggle();
    ButtonToggle toggleB = new ButtonToggle();
    ButtonToggle toggleLeftBumper = new ButtonToggle();

    //set up ftc dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void init(){
        robot.initNoGyro(hardwareMap);
    }

    public void init_loop(){
        if(gamepad1.left_stick_button)
            robot.lift.resetEncoder();
    }

    public void start(){}

    public void loop(){
        //Intake controls
        if(gamepad1.right_trigger > 0.5) robot.intake.on();
        else if(gamepad1.left_trigger > 0.5) robot.intake.reverse();
        else robot.intake.off();

        //Foundation gripper controls
        if(toggleX2.getState(gamepad1.x)) robot.foundationGripper.grab();
        else robot.foundationGripper.reset();

        //Lift controls
        //changing the set position
        if(toggleDpadUp.buttonPressed(gamepad1.dpad_up)) robot.lift.stageUp();
        else if(toggleDpadDown.buttonPressed(gamepad1.dpad_down)) robot.lift.stageDown();

        //toggle position correction on/off; it's on by default
        if(!toggleLeftBumper.getState(gamepad1.left_bumper) && !gamepad1.right_bumper) robot.lift.positionCorrection();
        else{
            //manual controls
            if (gamepad1.y && robot.lift.getMotorPosition() < robot.lift.MAX_HEIGHT) robot.lift.up();
            else if (gamepad1.a && robot.lift.getMotorPosition() > robot.lift.HOME_POSITION) robot.lift.down();
            else robot.lift.stop();
        }

        //reset lift encoders
        if(gamepad1.left_stick_button) robot.lift.resetEncoder();

        //grabber arm
        if(toggleB.getState(gamepad1.b)){
            robot.lift.moveOutsideRobot();
            telemetry.addLine("Arm Outside Robot");
        }
        else
            robot.lift.moveInsideRobot();

        if(!robot.lift.limitSwitch.getState()){
        }

        telemetry.addData("Lift power", robot.lift.liftMotor.getPower());
        telemetry.addData("Limit switch", robot.lift.limitSwitch.getState());
        telemetry.update();
        packet.put("Position", robot.lift.getMotorPosition());
        dashboard.sendTelemetryPacket(packet);
    }

}
