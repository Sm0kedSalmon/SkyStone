package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class Quarry extends LinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        //Deploy intake wheels
        robot.intake.intakeOn();
        sleep(100);
        robot.intake.intakeOff();

        robot.dt.strafeToPosition(10,SLOW_SPEED);

        robot.dt.driveToPosition(-12,SLOW_SPEED);

        //turn towards leftmost stone
        robot.gyroTurn(-52);

        //intakes left stone
        robot.intake.intakeOn();
        robot.dt.driveToPosition(30,SLOW_SPEED);
        robot.intake.intakeOff();
        robot.dt.driveToPosition(-15,SLOW_SPEED);

        //turns and moves under skybridge
        robot.gyroTurn(0);
        robot.dt.driveToPosition(58,FAST_SPEED);

        //releases stone
        robot.intake.intakeReverse();
        sleep(200);
        robot.intake.intakeOff();

        //zooms back
        robot.gyroTurn(0);
        robot.dt.driveToPosition(-58,FAST_SPEED);

        //turns towards
        robot.gyroTurn(-120);
        robot.intake.intakeOn();
        robot.dt.driveToPosition(15, SLOW_SPEED);
        robot.dt.driveToPosition(-15, SLOW_SPEED);
        robot.intake.intakeOff();
        robot.gyroTurn(0);

        robot.dt.driveToPosition(58,FAST_SPEED);
        robot.intake.intakeReverse();
        sleep(500);
        robot.intake.intakeOff();

        robot.gyroTurn(0);
        robot.dt.driveToPosition(-20,FAST_SPEED);
    }
}
