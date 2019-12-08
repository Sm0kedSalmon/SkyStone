package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.easyopencvtest.skystoneDetector;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous
@Disabled
public class LoadingZoneBlueIMU extends LinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;

    OpenCvCamera phoneCam;

    private final int rows = 640;
    private final int cols = 480;

    GroverHardware robot = new GroverHardware();
    public void runOpMode(){
        double skystonePosition = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new skystoneDetector.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.init(hardwareMap);

        while(!isStarted()) {
            if (skystoneDetector.valLeft == 0) skystonePosition = 0;
            else if (skystoneDetector.valMid == 0) skystonePosition = 1;
            else skystonePosition = 2;
        }

        waitForStart();

        //Deploy intake wheels
        robot.intake.on();
        sleep(100);
        robot.intake.off();

        robot.dt.strafeToPosition(10,SLOW_SPEED);

        robot.dt.driveToPosition(-12,SLOW_SPEED);

        //turn towards leftmost stone
        robot.gyroTurnPID(-52);

        //intakes left stone
        robot.intake.on();
        robot.dt.driveToPosition(30,SLOW_SPEED);
        robot.intake.off();
        robot.dt.driveToPosition(-15,SLOW_SPEED);

        telemetry.addData("Heading: ", robot.getHeading());

        //turns and moves under skybridge
        robot.gyroTurnPID(0);
        robot.dt.driveToPosition(58,FAST_SPEED);

        //releases stone
        robot.intake.reverse();
        sleep((long)RobotConstants.OUTTAKE_TIME);
        robot.intake.off();

        //zooms back
        robot.gyroTurnPID(0);
        robot.dt.driveToPosition(-58,FAST_SPEED);

        //picks up second stone
        robot.gyroTurnPID(-120);
        robot.intake.on();
        robot.dt.driveToPosition(15, SLOW_SPEED);
        robot.dt.driveToPosition(-15, SLOW_SPEED);
        robot.intake.off();
        robot.gyroTurnPID(0);

        robot.dt.driveToPosition(58,FAST_SPEED);
        robot.intake.reverse();
        sleep((long)RobotConstants.OUTTAKE_TIME);
        robot.intake.off();

        robot.gyroTurnPID(0);
        robot.dt.driveToPosition(-20,FAST_SPEED);
    }
}
