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

        while((int)(robot.getHeading()) < 90 && !isStopRequested()) {
            double c = robot.dt.gyroTurnCorrection(robot.getHeading(), 90);
            robot.dt.setMotorPower(-c, c, -c, c);
            telemetry.addData("Output: ", c);
            telemetry.update();
        }

        robot.dt.setMotorPower(0,0,0,0);
    }
}
