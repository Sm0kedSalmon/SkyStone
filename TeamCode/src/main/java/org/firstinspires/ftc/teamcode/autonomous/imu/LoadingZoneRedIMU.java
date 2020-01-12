package org.firstinspires.ftc.teamcode.autonomous.imu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dashboard.RobotConstants;
import org.firstinspires.ftc.teamcode.easyopencvtest.skystoneDetectorRed;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class LoadingZoneRedIMU extends LinearOpMode {
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
        phoneCam.setPipeline(new skystoneDetectorRed.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC

        robot.init(hardwareMap);

        while(!isStarted()) {
            if (skystoneDetectorRed.valLeft == 0) skystonePosition = 2;
            else if (skystoneDetectorRed.valMid == 0) skystonePosition = 1;
            else skystonePosition = 0;
            telemetry.addData("Position", skystonePosition);
            telemetry.update();
        }

        waitForStart();

        //Deploy intake wheels
        robot.intake.on();
        sleep(200);
        robot.intake.off();

        robot.lift.grabSkystone();

        //right
        if(skystonePosition == 0) {
            //strafe away from wall
            robot.strafeAndCorrectAngle(23.4, 0.3, 0);
            //align with left stone
            robot.driveAndCorrectAngle(-5.0, 0.3, 0);
            //turn to center skystone
            robot.gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            robot.driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            robot.driveAndCorrectAngle(RobotConstants.BACK_UP_DISTANCE, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(48.6, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.driveAndCorrectAngle(-70, 0.5, 180);
            robot.intake.off();
            //turn towards skystone
            robot.gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            robot.driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            robot.driveAndCorrectAngle(-18, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(70, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.driveAndCorrectAngle(-21.2, 0.5, 180);
            robot.intake.off();
        }

        //center
        else if(skystonePosition == 1){
            //strafe away from wall
            robot.strafeAndCorrectAngle(23.4, 0.3, 0);
            //align with center stone
            robot.driveAndCorrectAngle(3.0, 0.3, 0);
            //turn to center skystone
            robot.gyroTurnPID(-90);
            //intake center skystone
            robot.intake.on();
            robot.driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            robot.driveAndCorrectAngle(RobotConstants.BACK_UP_DISTANCE, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(48.6, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.driveAndCorrectAngle(-70, 0.5, 180);
            robot.intake.off();
            //turn towards skystone
            robot.gyroTurnPID(-90);
            //intake skystone
            robot.intake.on();
            robot.driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            robot.driveAndCorrectAngle(-18, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(70, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.driveAndCorrectAngle(-13.2, 0.5, 180);
            robot.intake.off();
        }

        //left
        else{
            //strafe away from wall
            robot.strafeAndCorrectAngle(23.4, 0.3, 0);
            //align with right skystone
            robot.driveAndCorrectAngle(9.0, 0.3, 0);
            //turn to right skystone
            robot.gyroTurnPID(-90);
            //intake right skystone
            robot.intake.on();
            robot.driveAndCorrectAngle(20, 0.3, -90);
            //drive back
            robot.driveAndCorrectAngle(RobotConstants.BACK_UP_DISTANCE, 0.3, -90);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(54.6, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //zoom back under skybridge, align with second skystone
            robot.driveAndCorrectAngle(-70, 0.5, 180);
            robot.intake.off();
            //turn towards stone TODO: make it so it intakes the right skystone instead of the center stone
            robot.gyroTurnPID(-45);
            //intake stone
            robot.intake.on();
            robot.dt.diagonalDriveNE(20, 0.3);
            //drive back
            robot.dt.diagonalDriveNE(-20, 0.3);
            robot.intake.off();
            //turn towards skybridge
            robot.gyroTurnPID(180);
            //drive under skybridge and release skystone
            robot.driveAndCorrectAngle(70, 0.5, 180);
            robot.intake.reverse();
            sleep(500);
            //park
            robot.driveAndCorrectAngle(-13.2, 0.5, 180);
            robot.intake.off();
        }






        //Deploy intake wheels
        /*robot.intake.on();
        sleep(100);
        robot.intake.off();

        robot.strafeAndCorrectAngle(15,SLOW_SPEED);
        robot.gyroTurnPID(180, SLOW_SPEED);
        robot.strafeAndCorrectAngle(10,SLOW_SPEED);

        robot.driveAndCorrectAngle(-12,SLOW_SPEED);

        //turn towards leftmost stone
        robot.gyroTurnPID(52, SLOW_SPEED);

        //intakes left stone
        robot.intake.on();
        robot.driveAndCorrectAngle(30,SLOW_SPEED);
        robot.intake.off();
        robot.driveAndCorrectAngle(-17,SLOW_SPEED);

        telemetry.addData("Heading: ", robot.getHeading());

        //turns and moves under skybridge
        robot.gyroTurnPID(-52, SLOW_SPEED);

        robot.driveAndCorrectAngle(58,FAST_SPEED);

        //releases stone
        robot.intake.reverse();
        sleep((long)RobotConstants.OUTTAKE_TIME);
        robot.intake.off();

        //zooms back
        robot.driveAndCorrectAngle(-58,FAST_SPEED);

        //picks up second stone
        robot.gyroTurnPID(120, SLOW_SPEED);
        robot.intake.on();
        robot.driveAndCorrectAngle(15, SLOW_SPEED);
        robot.driveAndCorrectAngle(-15, SLOW_SPEED);
        robot.intake.off();
        robot.gyroTurnPID(-120, SLOW_SPEED);

        //drops off second stone
        robot.driveAndCorrectAngle(58,FAST_SPEED);
        robot.intake.reverse();
        sleep((long)RobotConstants.OUTTAKE_TIME);
        robot.intake.off();

        //parks
        robot.driveAndCorrectAngle(-20,FAST_SPEED);*/

        //same program but with dashboard config variables

        /*robot.driveAndCorrectAngle(RobotConstants.QUARRY_2_DRIVE,SLOW_SPEED);

        //turn towards leftmost stone
        robot.gyroTurnPID(RobotConstants.QUARRY_3_TURN, SLOW_SPEED);

        //intakes left stone
        robot.intake.on();
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_4_INTAKE,SLOW_SPEED);
        robot.intake.off();
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_5_INTAKEREVERSE,SLOW_SPEED);

        telemetry.addData("Heading: ", robot.getHeading());

        //turns and moves under skybridge
        robot.gyroTurnPID(-RobotConstants.QUARRY_3_TURN, SLOW_SPEED);
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_6_ZOOM,FAST_SPEED);

        //releases stone
        robot.intake.reverse();
        sleep(200);
        robot.intake.off();

        //zooms back
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_7_ZOOM_BACK,FAST_SPEED);

        //picks up second stone
        robot.gyroTurnPID(RobotConstants.QUARRY_8_TURN2, SLOW_SPEED);
        robot.intake.on();
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_9_INTAKE2, SLOW_SPEED);
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_10_INTAKE2REVERSE, SLOW_SPEED);
        robot.intake.off();

        robot.gyroTurnPID(-RobotConstants.QUARRY_8_TURN2, SLOW_SPEED);
        robot.driveAndCorrectAngle(RobotConstants.QUARRY_11_ZOOM2,FAST_SPEED);
        robot.intake.reverse();
        sleep(500);
        robot.intake.off();

        robot.driveAndCorrectAngle(RobotConstants.QUARRY_12_PARK,FAST_SPEED);*/
    }
}
