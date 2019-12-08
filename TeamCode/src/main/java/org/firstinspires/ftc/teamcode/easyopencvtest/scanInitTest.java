package org.firstinspires.ftc.teamcode.easyopencvtest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.easyopencvtest.skystoneDetector;
import org.firstinspires.ftc.teamcode.robot.GroverHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class scanInitTest extends LinearOpMode {

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

        while(!isStarted()) {
            if (skystoneDetector.valLeft == 0) skystonePosition = 0;
            else if (skystoneDetector.valMid == 0) skystonePosition = 1;
            else skystonePosition = 2;
            telemetry.addData("Position:", skystonePosition);
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("Final Position:", skystonePosition);
        telemetry.update();

        sleep(500);
    }
}