package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class FoundationBlue extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        robot.dt.strafeToPosition(4, 0.25);
        robot.driveAndCorrectAngle(-RobotConstants.F_ALIGN_SIDEWAYS_DIST, 0.25, 0);
        robot.dt.strafeToPosition(RobotConstants.F_DRIVE_INTO_FOUND_DIST, 0.25);
        robot.foundationGripper.grab();
        sleep(500);
        robot.dt.strafeToPosition(RobotConstants.F_BACK_UP_DIST, 0.75);
        robot.gyroTurnPID(90);
        robot.foundationGripper.reset();
        sleep(500);
        robot.strafeAndCorrectAngle(RobotConstants.F_PARK_DIST,0.25, -90);

    }
}