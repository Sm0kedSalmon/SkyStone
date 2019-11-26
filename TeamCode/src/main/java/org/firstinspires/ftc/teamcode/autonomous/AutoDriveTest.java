package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class AutoDriveTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        robot.encoderTurn(50,0.5);
        robot.encoderTurn(0,0.5);

        robot.dt.setMotorPower(0,0,0,0);
    }
}