package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@TeleOp(name = "TensorFlow Test")

public class TensorFlowTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AeeJaAf/////AAABmXrchGwLg0pXiPVAgcIM5ayAn2B3H+06gM6QUWQD7yj0kOlTcN/rfpseVxZM53CM79USal/Js9+Pk0iYR2nmNlwP3KHx1DFxjBAhH7409SBhhdYCRdeS3ZL4z5aV0woEL8Iqu5rVQe+4K11b9rUVvdMWj5AUAp/f4HN2Uct2DU56WJhZ+h/DBp3e8vXWU1MdLX0GsitTt7qpshP+uvkwUa8QQ36TBvP0GSyY0Df9O52e2HuAUL9ruQs4h3GZ82i50oRXu20MQ//3XXmBH0y/DMZ0nm8jQ1OIySmEzGAWtXK/e9/I9VRZ2ilT8Sh5rAJoYUG1o64pzNweGPwoRqohYnIoXQ29meA7I6cN3N1U3E7R";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // tells the program the positions of each of the stones
                        float skyStoneX = -1;
                        float stone1X = -1;
                        float stone2X = -1;
                        for(Recognition recognition : updatedRecognitions){
                            if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                                skyStoneX = recognition.getBottom();
                            }
                            else if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
                                if(stone1X == -1){
                                    stone1X = recognition.getBottom();
                                }
                                else
                                    stone2X = recognition.getBottom();
                            }
                        }

                        if(skyStoneX == -1) telemetry.addLine("Skystone position: Right");
                        else if(skyStoneX > stone1X) telemetry.addLine("Skystone position: Center");
                        else telemetry.addLine("Skystone position: Left");

                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
