package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;


@Autonomous(name = "Regular Drive Test", group="Test")
public class DriveTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        ElapsedTime time = new ElapsedTime();
        time.reset();
        driveToPosition(40,1);
        sleep(500);

        double distance = robot.dt.getAveragePosition() / robot.dt.TICKS_PER_INCH;
        double avgVelocity = distance / time.seconds();

        telemetry.addData("Distance", distance);
        telemetry.addData("Average Velocity", avgVelocity);
        telemetry.update();

        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}