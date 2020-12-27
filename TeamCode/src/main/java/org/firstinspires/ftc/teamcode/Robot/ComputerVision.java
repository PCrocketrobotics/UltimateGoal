package org.firstinspires.ftc.teamcode.Robot;

import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ComputerVision {
    final Robot robot;
    public static final String VUFORIA_KEY =
            "ASw5gYz/////AAABmcO0k7lgZkVatj0ceEv9NJlY1VGi+pv6kKpFTFwMDup4i496GVQyd7fT16wsuGUD8th+AzzfjsKYU7XrqFpaMwdafjIjsXa5FExX+il+KHntcQ68Jb4OvuB3S4/2Qoj22L4nlqGHH970MfaXzQjFJPUIEPUowQgXVdc6JzgPPNd020+ZGz3u4BpLsMAnb7BEf9IytlbXv6B+esOfIOScgnrue+AsAyIcTWlzYzJiz+Wn596IPTrcTDVmi+1g9jIqEiCkFMIqxXTNukCIhRk8STE+nCdfv5hr7k0tfUfMCx6hHQoJK9TH3mqfNLiWszH3w/lwOl1fXVRFTeKCs3fXZA5JdDLSwp7ZQ9P3UJkCHGTM";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String LABEL_NO_ELEMENT = "None";

    public ComputerVision(Robot robot) {this.robot = robot;}

    public void init() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = robot.opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void activate() {
        if (tfod != null) {
           tfod.activate();
        }
       tfod.setZoom(2, 1.78);
    }

    public String detect() {
        String value = "None";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                robot.opMode.telemetry.addData("# Objects Detected", recognitions.size());
                int i = 0;
                for (Recognition recognition : recognitions) {
                    robot.opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    robot.opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    robot.opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",  recognition.getRight(), recognition.getBottom());
                    value = recognition.getLabel();
                }
            }
        }
        return  value;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
