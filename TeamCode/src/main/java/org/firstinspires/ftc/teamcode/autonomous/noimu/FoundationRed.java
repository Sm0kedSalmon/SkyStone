package org.firstinspires.ftc.teamcode.autonomous.noimu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
@Autonomous
public class FoundationRed extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();


        /*robot.dt.strafeToPosition(4, 0.25);
        robot.dt.driveToPosition(-11, 0.25);
        robot.dt.strafeToPosition(28, 0.25);
        robot.gripper.grab();
        sleep(500);
        robot.dt.strafeToPosition(-36, 0.5);
        robot.gripper.reset();
        sleep(500);
        robot.dt.driveToPosition(52, 0.25);*/
    }
}