package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name="test")
public class AutoDriveTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        robot.dt.strafeToPosition(6,0.2);

        //robot.dt.driveAtAngle(4,-10,0.2);

        //robot.gyroTurn(90);

        robot.dt.setMotorPower(0,0,0,0);
    }
}
