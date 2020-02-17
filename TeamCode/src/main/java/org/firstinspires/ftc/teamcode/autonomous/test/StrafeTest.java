package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;


@Autonomous(name = "Strafe Test", group="Test")
public class StrafeTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        strafeAndCorrectAngle(20,0.3,0);
        sleep(500);

        telemetry.addData("Heading", robot.imu.getHeading());
        telemetry.update();

        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}