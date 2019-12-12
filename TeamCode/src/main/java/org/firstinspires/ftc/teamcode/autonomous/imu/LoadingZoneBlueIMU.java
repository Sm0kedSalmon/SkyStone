package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.usb.exception.RobotUsbTimeoutException;
import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.easyopencvtest.skystoneDetector;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class LoadingZoneBlueIMU extends LinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    OpenCvCamera phoneCam;

    private final int rows = 640;
    private final int cols = 480;

    GroverHardware robot = new GroverHardware();

    public void runOpMode() {
        double skystonePosition = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new skystoneDetector.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.init(hardwareMap);

        while (!isStarted()) {
            if (skystoneDetector.valLeft == 0) skystonePosition = 0; //left
            else if (skystoneDetector.valMid == 0) skystonePosition = 1; //center
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
        if (skystonePosition == 0) {
            //strafe away from wall
            robot.dt.strafeToPosition(23.4, SLOW_SPEED);
            //align with center stone
            robot.dt.driveToPosition(5.0, SLOW_SPEED);
            //turn to center skystone
            robot.gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, SLOW_SPEED);
            //drive back
            robot.dt.driveToPosition(-22, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(39.6, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.dt.driveToPosition(-62, FAST_SPEED);
            robot.intake.off();
            //turn towards skystone
            robot.gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, SLOW_SPEED);
            //drive back
            robot.dt.driveToPosition(-21, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(62, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.dt.driveToPosition(-13.2, FAST_SPEED);
            robot.intake.off();
        }

        //center
        else if (skystonePosition == 1) {
            //strafe away from wall
            robot.dt.strafeToPosition(23.4, SLOW_SPEED);
            //align with center stone
            robot.dt.driveToPosition(-2.0, SLOW_SPEED);
            //turn to center skystone
            robot.gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, SLOW_SPEED);
            //drive back
            robot.dt.driveToPosition(-22, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(47.6, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.dt.driveToPosition(-70, FAST_SPEED);
            robot.intake.off();
            //turn towards skystone
            robot.gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, SLOW_SPEED);
            //drive back
            robot.dt.driveToPosition(-21, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(70, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.dt.driveToPosition(-13.2, FAST_SPEED);
            robot.intake.off();
        }

        //right
        else {
            //strafe away from wall
            robot.dt.strafeToPosition(23.4, 0.3);
            //align with right skystone
            robot.dt.driveToPosition(-9.0, 0.3);
            //turn to right skystone
            robot.gyroTurnPID(-90);
            //intake right skystone
            robot.intake.on();
            robot.dt.driveToPosition(20, 0.3);
            //drive back
            robot.dt.driveToPosition(-22, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(54.6, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.dt.driveToPosition(-70, FAST_SPEED);
            robot.intake.off();
            //turn towards stone TODO: make it so it intakes the right skystone instead of the center stone
            robot.gyroTurnPID(-90);
            //intake stone
            robot.intake.on();
            robot.dt.driveToPosition(20, SLOW_SPEED);
            //drive back
            robot.dt.driveToPosition(-21, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(0);
            //drive under skybridge and release skystone
            robot.dt.driveToPosition(70, FAST_SPEED);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.dt.driveToPosition(-13.2, FAST_SPEED);
            robot.intake.off();
        }
    }
}