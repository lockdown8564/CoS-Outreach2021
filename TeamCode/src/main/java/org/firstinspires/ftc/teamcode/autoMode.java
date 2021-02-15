package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "autonomous", group = "Auto")
public class autoMode extends LinearOpMode {

    private Hardware robot = new Hardware();

    OpenCvCamera webCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ringStackSizeDetector detector = new ringStackSizeDetector(telemetry);
        webCam.setPipeline(detector);
        webCam.openCameraDevice();
        webCam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        switch (detector.getStackSize()) {
            case NONE:
                // ...
                break;
            case ONE:
                // ...
                break;
            case FOUR:
                // ...
        }
        webCam.stopStreaming();
    }
}