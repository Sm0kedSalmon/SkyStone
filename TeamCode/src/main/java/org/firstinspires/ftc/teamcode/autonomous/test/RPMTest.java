package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Disabled
@Autonomous(name = "RPM Test", group="Test")
public class RPMTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        ElapsedTime t = new ElapsedTime();
        t.reset();
        robot.dt.setMotorPower(1,1,1,1);
        while(t.seconds() < 60){
            telemetry.addData("Time Left", 60 - t.seconds());
            telemetry.update();
        }

        double distance = robot.dt.getAveragePosition() / robot.dt.TICKS_PER_INCH;
        double avgVelocity = distance / t.seconds();
        double RPM = robot.dt.getAveragePosition() / robot.dt.TICKS_PER_MOTOR_REV;

        telemetry.addData("Distance", distance);
        telemetry.addData("Average Velocity", avgVelocity);
        telemetry.addData("Average RPM", RPM);
        telemetry.update();
        sleep(500);
    }
}