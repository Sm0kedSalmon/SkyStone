package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "RPM Test", group="Test")
public class RPMTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
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