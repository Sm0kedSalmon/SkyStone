package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Disabled
@Autonomous(name = "Strafe Test", group="Test")
public class StrafeTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        robot.strafeAndCorrectAngle(20,0.3,0);
        sleep(500);

        telemetry.addData("Heading", robot.getHeading());
        telemetry.update();

        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}