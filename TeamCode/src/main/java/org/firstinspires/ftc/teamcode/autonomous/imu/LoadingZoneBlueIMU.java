package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import easyOpenCV.skystoneDetectorBlue;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class LoadingZoneBlueIMU extends HydrofluoricLinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    OpenCvCamera phoneCam;

    private final int rows = 640;
    private final int cols = 480;

    GroverHardware robot = new GroverHardware();

    public void runOpMode() {
        super.initHardware(robot);
        double skystonePosition = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new skystoneDetectorBlue.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.init(hardwareMap);

        while (!isStarted()) {
            if (skystoneDetectorBlue.valLeft == 0) skystonePosition = 0; //left
            else if (skystoneDetectorBlue.valMid == 0) skystonePosition = 1; //center
            else skystonePosition = 2; //right
            telemetry.addData("Position", skystonePosition);
            telemetry.update();
        }

        waitForStart();

        //Deploy intake wheels
        robot.intake.intakeLeft.setPower(1);
        robot.intake.intakeRight.setPower(1);
        sleep(200);
        robot.intake.off();

        robot.lift.grabSkystone();

        //left
        if (skystonePosition == 0) {
            //strafe away from wall
            strafeToPosition(23.4, 0.3);
            //align with center stone
            driveAndCorrectAngle(6.0, SLOW_SPEED, 0);
            //turn to center skystone
            gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            driveAndCorrectAngle(20, SLOW_SPEED, -90);
            //drive back
            driveAndCorrectAngle(-17, SLOW_SPEED, -90);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(39.6, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            driveAndCorrectAngle(-63, FAST_SPEED, 0);
            robot.intake.off();
            //strafe to give robot space
            strafeAndCorrectAngle(-4, 0.3, 0);
            //turn towards skystone
            gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            driveAndCorrectAngle(20, SLOW_SPEED, -90);
            //drive back
            driveAndCorrectAngle(-17, SLOW_SPEED, -90);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(62, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //park
            driveAndCorrectAngle(-13.2, FAST_SPEED, 0);
            robot.intake.off();
        }

        //center
        else if (skystonePosition == 1) {
            //strafe away from wall
            strafeToPosition(23.4, 0.3);
            //align with center stone
            driveAndCorrectAngle(-2.0, SLOW_SPEED, 0);
            //turn to center skystone
            gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            driveAndCorrectAngle(20, SLOW_SPEED, -90);
            //drive back
            driveAndCorrectAngle(-17, SLOW_SPEED, -90);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(47.6, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            driveAndCorrectAngle(-70, FAST_SPEED, 0);
            robot.intake.off();
            //strafe to give robot space
            strafeAndCorrectAngle(-4, 0.3, 0);
            //turn towards skystone
            gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            driveAndCorrectAngle(20, SLOW_SPEED, -90);
            //drive back
            driveAndCorrectAngle(-17, SLOW_SPEED, -90);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(70, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //park
            driveAndCorrectAngle(-13.2, FAST_SPEED, 0);
            robot.intake.off();
        }

        //right
        else {
            //strafe away from wall
            strafeToPosition(23.4, 0.3);
            //align with right skystone
            driveAndCorrectAngle(-9.0, 0.3, 0);
            //turn to right skystone
            gyroTurnPID(-90);
            //intake right skystone
            robot.intake.on();
            driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            driveAndCorrectAngle(-17, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(54.6, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            driveAndCorrectAngle(-70, FAST_SPEED, 0);
            robot.intake.off();
            //strafe to give robot space
            strafeAndCorrectAngle(-4, 0.3, 0);
            //turn towards stone
            gyroTurnPID(-135);
            //intake stone
            robot.intake.on();
            diagonalDriveNW(20, SLOW_SPEED);
            //drive back
            diagonalDriveNW(-20, SLOW_SPEED);
            robot.intake.off();
            //turn towards skybridge
            gyroTurnPID(0);
            //drive under skybridge and release skystone
            driveAndCorrectAngle(70, FAST_SPEED, 0);
            robot.intake.reverse();
            sleep(500);
            //park
            driveAndCorrectAngle(-13.2, FAST_SPEED, 0);
            robot.intake.off();
        }
    }
}