package org.firstinspires.ftc.teamcode.autonomous.noimu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.easyopencvtest.skystoneDetectorBlue;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
@Autonomous
public class LoadingZoneBlue extends LinearOpMode {
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
        phoneCam.setPipeline(new skystoneDetectorBlue.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.initNoGyro(hardwareMap);

        while(!isStarted()) {
            if (skystoneDetectorBlue.valLeft == 0) skystonePosition = 0; //left
            else if (skystoneDetectorBlue.valMid == 0) skystonePosition = 1; //center
            else skystonePosition = 2; //right
            telemetry.addData("Position", skystonePosition);
            telemetry.update();
        }

        waitForStart();

        //Deploy intake wheels
        robot.intake.on();
        sleep(100);
        robot.intake.off();

        //left
        if(skystonePosition == 0) {
            robot.dt.strafeToPosition(10,SLOW_SPEED);
            robot.dt.driveToPosition(-12, SLOW_SPEED);

            //turn towards leftmost stone
            robot.dt.encoderTurn(-52, SLOW_SPEED);

            //intakes left stone
            robot.intake.on();
            robot.dt.driveToPosition(30, SLOW_SPEED);
            robot.intake.off();
            robot.dt.driveToPosition(-17, SLOW_SPEED);

            //turns and moves under skybridge
            robot.dt.encoderTurn(52, SLOW_SPEED);

            robot.dt.driveToPosition(58, FAST_SPEED);

            //releases stone
            robot.intake.reverse();
            sleep((long) RobotConstants.OUTTAKE_TIME);
            robot.intake.off();

            //zooms back
            robot.dt.driveToPosition(-58, FAST_SPEED);

            //picks up second stone
            robot.dt.encoderTurn(-120, SLOW_SPEED);
            robot.intake.on();
            robot.dt.driveToPosition(15, SLOW_SPEED);
            robot.dt.driveToPosition(-13, SLOW_SPEED);
            robot.intake.off();
            robot.dt.encoderTurn(120, SLOW_SPEED);

            //drops off second stone
            robot.dt.driveToPosition(58, FAST_SPEED);
            robot.intake.reverse();
            sleep((long) RobotConstants.OUTTAKE_TIME);
            robot.intake.off();

            //parks
            robot.dt.driveToPosition(-20, FAST_SPEED);
        }

        //center
        else if(skystonePosition == 1){
            //strafe away from wall
            robot.dt.strafeToPosition(23.4, 0.3);
            //align with center stone
            robot.dt.driveToPosition(-2.0, 0.3);
            //turn to center skystone
            robot.dt.encoderTurn(-90, 0.3);
            //intake center skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, 0.3);
            //drive back
            robot.dt.driveToPosition(-22, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.dt.encoderTurn(90, 0.3);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(47.6, 0.5);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.dt.driveToPosition(-70, 0.5);
            robot.intake.off();
            //turn towards skystone
            robot.dt.encoderTurn(-90, 0.3);
            //intake skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, 0.3);
            //drive back
            robot.dt.driveToPosition(-21, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.dt.encoderTurn(90, 0.3);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(70, 0.5);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.dt.driveToPosition(-13.2, 0.5);
            robot.intake.off();
        }

        //right
        else{
            //strafe away from wall
            robot.dt.strafeToPosition(23.4, 0.3);
            //align with right skystone
            robot.dt.driveToPosition(-9.0, 0.3);
            //turn to right skystone
            robot.dt.encoderTurn(-90, 0.3);
            //intake right skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, 0.3);
            //drive back
            robot.dt.driveToPosition(-22, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.dt.encoderTurn(90, 0.3);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(54.6, 0.5);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.dt.driveToPosition(-70, 0.5);
            robot.intake.off();
            //turn towards stone TODO: make it so it intakes the right skystone instead of the center stone
            robot.dt.encoderTurn(-90, 0.3);
            //intake stone
            robot.intake.on();
            robot.dt.driveToPosition(20, 0.3);
            //drive back
            robot.dt.driveToPosition(-21, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.dt.encoderTurn(90, 0.3);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(70, 0.5);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.dt.driveToPosition(-13.2, 0.5);
            robot.intake.off();
        }
    }
}