package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "Motion Profiling Test", group="Test")
public class MotionProfileTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        robot.timeMotionProfile(RobotConstants.DISTANCE);

        telemetry.addData("Angle offset", robot.getHeading());
        telemetry.addData("Distance", robot.dt.FLMotor.getCurrentPosition() / robot.dt.TICKS_PER_INCH);
        telemetry.update();
        sleep(500);

        robot.timeMotionProfile(-RobotConstants.DISTANCE);


    }
}