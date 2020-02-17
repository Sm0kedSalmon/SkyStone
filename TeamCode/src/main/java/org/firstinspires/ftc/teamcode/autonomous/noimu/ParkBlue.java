package org.firstinspires.ftc.teamcode.autonomous.noimu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous
public class ParkBlue extends HydrofluoricLinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    OpenCvCamera phoneCam;

    private final int rows = 640;
    private final int cols = 480;

    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        super.initHardware(robot);
        robot.init(hardwareMap);


        waitForStart();

        //Deploy intake wheels
        robot.intake.on();
        sleep(100);
        robot.intake.off();
        driveAndCorrectAngle(-15,SLOW_SPEED,0);
    }
}