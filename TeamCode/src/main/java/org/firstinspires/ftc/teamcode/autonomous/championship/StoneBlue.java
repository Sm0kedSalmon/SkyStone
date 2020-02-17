package org.firstinspires.ftc.teamcode.autonomous.championship;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.HydrofluoricLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;

import dashboard.RobotConstants;
/*import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;*/

@Autonomous
public class StoneBlue extends HydrofluoricLinearOpMode {
    public static final double SLOW_SPEED = 0.3;
    public static final double FAST_SPEED = 0.75;
    //OpenCvCamera phoneCam;

    /*private final int rows = 640;
    private final int cols = 480;*/

    GroverHardware robot = new GroverHardware();

    public void runOpMode() {
        super.initHardware(robot);

        double skystonePosition = 0;

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new skystoneDetectorBlue.StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC*/

        robot.init(hardwareMap);

        /*while (!isStarted() && !isStopRequested()) {
            if (skystoneDetectorBlue.valLeft == 0) skystonePosition = 0; //left
            else if (skystoneDetectorBlue.valMid == 0) skystonePosition = 1; //center
            else skystonePosition = 2; //right
            telemetry.addData("Position", skystonePosition);
            telemetry.update();
        }*/

        waitForStart();

        //Deploy intake wheels
        robot.intake.intakeLeft.setPower(1);
        robot.intake.intakeRight.setPower(1);
        sleep(200);
        robot.intake.off();

        robot.lift.grabSkystone();

        //left (far from wall)
        if (skystonePosition == 0) {
            //short strafe to align vertically with leftmost skystone
            strafeAndCorrectAngle(RobotConstants.SB_STRAFE_OFF_WALL,SLOW_SPEED,0);
            //turn 45 degrees
            gyroTurnPID(-45);
            robot.intake.on();
            //intake first stone
            diagonalDriveNE(RobotConstants.SB_ALIGN_WITH_QUARRY,0.5);
            straightMotionProfile(RobotConstants.SB_INTAKE_FIRST_STONE,-45);
            straightMotionProfile(-RobotConstants.SB_INTAKE_FIRST_STONE,-45);
            //align with skybridge
            diagonalDriveNE(RobotConstants.SB_ALIGN_WITH_SKYBRIDGE, -FAST_SPEED);
            //turn to face skybridge
            gyroTurnPID(0);
            robot.intake.off();

            //align with foundation
            straightMotionProfile(RobotConstants.SB_ALIGN_WITH_FOUNDATION,0);
            //move against foundation and raise lift
            robot.lift.setCurrentPosition(4);
            strafeToPosition(RobotConstants.SB_MOVE_AGAINST_FOUNDATION,SLOW_SPEED);
            //grab foundation
            robot.lift.movetoFoundationDeposit();
            robot.foundationGripper.grab();
            sleep(300);
            robot.lift.releaseSkystone();
            sleep(200);
            //drag foundation towards building zone
            robot.lift.moveInsideRobot();
            strafeToPosition(RobotConstants.SB_DRAG_FOUNDATION,0.4);
            //turn foundation against wall
            robot.lift.setCurrentPosition(0);
            gyroTurnPID(90);
            robot.foundationGripper.reset();
            //slam foundation
            strafeToPosition(20,FAST_SPEED);

            //align with skybridge a second time
            gyroTurnPID(180);
            diagonalDriveNW(RobotConstants.SB_ALIGN_WITH_SKYBRIDGE_2,FAST_SPEED);
            gyroTurnPID(180);
            //zoom under skybridge to align with second stone
            straightMotionProfile(RobotConstants.SB_ZOOM_TO_SECOND_SKYSTONE,180);
            //strafe to align with skystones and pick up second skystone
            strafeAndCorrectAngle(-RobotConstants.SB_ALIGN_WITH_SECOND_SKYSTONE, SLOW_SPEED, 0);
            robot.intake.on();
            straightMotionProfile(RobotConstants.SB_PICK_UP_SECOND_STONE, -135);
            diagonalDriveNW(-RobotConstants.SB_ALIGN_WITH_SKYBRIDGE_FINAL, SLOW_SPEED);
            robot.intake.off();
            //turn to skybridge, facing backwards
            //zoom under skybridge a second time
            straightMotionProfile(-RobotConstants.SB_ZOOM_UNDER_SKYBRIDGE_FINAL, 180);
            //park
            straightMotionProfile(RobotConstants.SB_PARK, 180);

            //drive under skybridge to align w/foundation, and halfway through, start a lift PID loop to the right height
            //regular turn to 45 degree angle
            /*
            //when lift is at right height, rotate arm to be above foundation
            robot.lift.rotator.setPosition(1);
            sleep(300);
            //drop stone onto foundation
            robot.lift.releaseSkystone();
            sleep(1000);
            //rotate arm back
            robot.lift.moveInsideRobot();
            sleep(200);
            //drive back under skybridge to align with second stone
            robot.lift.setCurrentPosition(0);
            straightMotionProfile(-87,0);
            //turn 90 degrees
            gyroTurn(-100,SLOW_SPEED);
            //plow through quarry to intake stone
            robot.intake.on();
            straightMotionProfile(15,-100);
            robot.lift.grabSkystone();
            //reverse direction to align with center of lane
            straightMotionProfile(-15,-100);
            robot.intake.off();
            //PID turn to face skybridge
            gyroTurnPID(0);
            //drive under skybridge to align w/foundation, and halfway through, start a lift PID loop to the right height
            robot.lift.setCurrentPosition(4);
            straightMotionProfile(87,0);
            //when lift is at right height, rotate arm to be above foundation
            robot.lift.rotator.setPosition(1);
            sleep(300);
            //drop stone onto foundation
            robot.lift.releaseSkystone();
            sleep(1000);
            //rotate arm back
            robot.lift.moveInsideRobot();
            sleep(200);
            //move robot against foundation
            //grab foundation
            //drag foundation sideways
            //PID turn to 90 degrees
            //slam foundation against wall
            //diagonal drive to align with lane
            //park under skybridge*/
        }

        //center
        else if (skystonePosition == 1) {

        }

        //right (close to wall)
        else {

        }
    }
}