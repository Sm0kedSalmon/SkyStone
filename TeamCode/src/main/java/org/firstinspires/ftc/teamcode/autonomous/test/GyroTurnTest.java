package org.firstinspires.ftc.teamcode.autonomous.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous(name = "Gyro Turn Test")
public class GyroTurnTest extends LinearOpMode {
    GroverHardware robot = new GroverHardware();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        while(robot.getHeading() != 90 && !isStopRequested()) {
            double c = robot.dt.gyroTurnCorrection(robot.getHeading(), 90, robot.dt.turnToAnglePID);
            robot.dt.setMotorPower(-c, c, -c, c);
            packet.put("Error", robot.dt.turnToAnglePID.getTarget() - robot.dt.turnToAnglePID.getCurrent());
            dashboard.sendTelemetryPacket(packet);
        }
            robot.dt.setMotorPower(0,0,0,0);


    }
}