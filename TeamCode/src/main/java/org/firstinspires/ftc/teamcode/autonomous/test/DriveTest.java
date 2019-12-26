package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "Regular Drive Test", group="Test")
public class DriveTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        robot.dt.driveToPosition(30, 0.5);
        telemetry.addData("Heading offset", robot.getHeading());
        telemetry.update();
        sleep(500);
        //robot.dt.driveToPosition(-50, 0.5);


        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}