package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
@Autonomous
public class FoundationRed extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        waitForStart();


        strafeToPosition(4, 0.25);
        driveAndCorrectAngle(-14, 0.25, 0);
        strafeToPosition(30, 0.25);
        robot.foundationGripper.grab();
        sleep(750);
        strafeToPosition(-70, 0.75);
        gyroTurnPID(-90);
        robot.foundationGripper.reset();
        sleep(500);
        strafeToPosition(10, 0.75);
        strafeToPosition(-10, 0.75);
        driveToPosition(-30, 0.75);
        driveAndCorrectAngle(4, 0.25, 0);
        strafeAndCorrectAngle(-40,0.25, -90);

    }
}