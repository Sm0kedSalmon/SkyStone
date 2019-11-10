package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

@Autonomous
public class Quarry extends LinearOpMode {
    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        //drive in front of quarry
        robot.dt.driveStraight(23, 0.5);

        //scanning goes here probably

        //turn to stone
        robot.gyroTurn(-10);

        //pick up stone
        robot.intake.intakeOn();
        robot.dt.driveStraight(18, 0.25);
        sleep(500);

        //move back
        robot.dt.driveStraight(-16, 0.25);
        robot.intake.intakeOff();

        //turn to face skybridge
        robot.gyroTurn(90);

        //go completely under skybridge
        robot.dt.driveStraight(48, 0.5);

        //release stone
        robot.intake.intakeReverse();
        sleep(500);

        //Reverse, reverse!
        robot.dt.driveStraight(-65,0.5);

        //turn towards second stone
        robot.gyroTurn(45);

        //pick up second stone
        robot.intake.intakeOn();
        robot.dt.driveStraight(18.5, 0.25);
        sleep(500);

        //move back
        robot.dt.driveStraight(-18.5, 0.25);
        robot.intake.intakeOff();

        //turn to face skybridge
        robot.gyroTurn(90);

        //go completely under skybridge
        robot.dt.driveStraight(65, 0.5);

        //drop off second stone
        robot.intake.intakeReverse();
        sleep(500);

        //park
        robot.dt.driveStraight(-16, 0.5);
    }
}
