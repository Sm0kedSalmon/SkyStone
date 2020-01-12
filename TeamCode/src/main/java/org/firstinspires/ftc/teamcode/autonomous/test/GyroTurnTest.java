package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Disabled
@Autonomous(name = "Gyro Turn Test", group="Test")
public class GyroTurnTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();
        //Math.abs(robot.getHeading() - RobotConstants.TARGET_ANGLE) > RobotConstants.THRESHOLD
        while(!isStopRequested() && Math.abs(robot.getHeading() - RobotConstants.TARGET_ANGLE) > RobotConstants.THRESHOLD) {
            double c = robot.dt.gyroPIDCorrection(robot.getHeading(), RobotConstants.TARGET_ANGLE, robot.dt.turnToAnglePIDTest);
            robot.dt.setMotorPower(-c, c, -c, c);

            packet.put("Current angle", robot.getHeading());
            packet.put("Error", robot.dt.turnToAnglePIDTest.getTarget() - robot.dt.turnToAnglePIDTest.getCurrent());
            dashboard.sendTelemetryPacket(packet);
        }

        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}