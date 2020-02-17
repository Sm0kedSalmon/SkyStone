package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dashboard.RobotConstants;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class FoundationBlue extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        waitForStart();

        strafeToPosition(4, 0.25);
        driveAndCorrectAngle(-RobotConstants.F_ALIGN_SIDEWAYS_DIST, 0.25, 0);
        strafeToPosition(RobotConstants.F_DRIVE_INTO_FOUND_DIST, 0.25);
        robot.foundationGripper.grab();
        sleep(750);
        strafeToPosition(RobotConstants.F_BACK_UP_DIST, 0.75);
        gyroTurnPID(90);
        robot.foundationGripper.reset();
        sleep(500);
        strafeToPosition(10, 0.75);
        strafeToPosition(-10, 0.75);
        //robot.gyroTurnPID(180);
        driveToPosition(30, 0.5);
        driveAndCorrectAngle(-4, 0.25, 0);
        strafeAndCorrectAngle(RobotConstants.F_PARK_DIST,0.25, 90);

    }
}