package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dashboard.RobotConstants;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "Motion Profiling Test", group="Test")
public class MotionProfileTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        super.initHardware(robot);

        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();

        straightMotionProfile(RobotConstants.DISTANCE, 0);

        telemetry.addData("Angle offset", robot.imu.getHeading());
        telemetry.addData("Distance", robot.dt.FLMotor.getCurrentPosition() / robot.dt.TICKS_PER_INCH);
        telemetry.update();
        sleep(500);

        straightMotionProfile(-RobotConstants.DISTANCE, 0);


    }
}