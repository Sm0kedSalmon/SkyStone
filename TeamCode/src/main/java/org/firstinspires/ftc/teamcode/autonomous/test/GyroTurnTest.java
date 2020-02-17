package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dashboard.RobotConstants;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "Gyro Turn Test", group="Test")
public class GyroTurnTest extends HydrofluoricLinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    ElapsedTime time = new ElapsedTime();

    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);
        robot.dt.resetEncoders();
        waitForStart();
        time.reset();
        //Math.abs(robot.getHeading() - RobotConstants.TARGET_ANGLE) > RobotConstants.GYRO_TURN_THRESHOLD
        while(!isStopRequested() && Math.abs(robot.imu.getHeading() - RobotConstants.TARGET_ANGLE) > RobotConstants.GYRO_TURN_THRESHOLD) {
            double c = gyroPIDCorrection(robot.imu.getHeading(), RobotConstants.TARGET_ANGLE, robot.dt.turnToAnglePIDTest);
            robot.dt.setMotorPower(-c, c, -c, c);

            packet.put("Current angle", robot.imu.getHeading());
            packet.put("Error", robot.dt.turnToAnglePIDTest.getTarget() - robot.dt.turnToAnglePIDTest.getCurrent());
            packet.put("Time", time.milliseconds() / 1000.0);
            dashboard.sendTelemetryPacket(packet);
        }

        //turn to center skystone
        //robot.gyroTurnPID(90);
    }
}