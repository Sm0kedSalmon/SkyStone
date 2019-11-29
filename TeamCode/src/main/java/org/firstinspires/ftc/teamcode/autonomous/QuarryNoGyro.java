package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class QuarryNoGyro extends LinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        //Deploy intake wheels
        robot.intake.on();
        sleep(100);
        robot.intake.off();

        robot.dt.strafeToPosition(10,SLOW_SPEED);

        robot.dt.driveToPosition(-12,SLOW_SPEED);

        //turn towards leftmost stone
        robot.dt.encoderTurn(-52, SLOW_SPEED);

        //intakes left stone
        robot.intake.on();
        robot.dt.driveToPosition(30,SLOW_SPEED);
        robot.intake.off();
        robot.dt.driveToPosition(-15,SLOW_SPEED);

        telemetry.addData("Heading: ", robot.getHeading());

        //turns and moves under skybridge
        robot.dt.encoderTurn(52, SLOW_SPEED);

        robot.dt.driveToPosition(58,FAST_SPEED);

        //releases stone
        robot.intake.reverse();
        sleep(200);
        robot.intake.off();

        //zooms back
        robot.dt.driveToPosition(-58,FAST_SPEED);

        //picks up second stone
        robot.dt.encoderTurn(-120, SLOW_SPEED);
        robot.intake.on();
        robot.dt.driveToPosition(15, SLOW_SPEED);
        robot.dt.driveToPosition(-15, SLOW_SPEED);
        robot.intake.off();
        robot.dt.encoderTurn(120, SLOW_SPEED);

        //drops off second stone
        robot.dt.driveToPosition(58,FAST_SPEED);
        robot.intake.reverse();
        sleep(500);
        robot.intake.off();

        //parks
        robot.dt.driveToPosition(-20,FAST_SPEED);
    }
}