package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;


@Autonomous(name = "Heading Correction Test", group="Test")
public class HeadingCorrectionTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        driveAndCorrectAngle(50,0.5,0);
        sleep(500);
        driveAndCorrectAngle(-50,0.5,0);
    }
}