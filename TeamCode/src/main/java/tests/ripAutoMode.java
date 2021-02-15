package tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autoMode;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "auto", group = "Auto")
public class ripAutoMode extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    autoMode.DeterminationPipeline pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                    getIdentifier("cameraMonitorViewId","id",
                            hardwareMap.appContext.getPackageName());
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera
                    (OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            StackSizeDetector detector = new StackSizeDetector(telemetry);
            phoneCam.setPipeline(detector);
            phoneCam.openCameraDeviceAsync(
                    () -> phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT)
            );

            waitForStart();
            switch(detector.getStackSize()) {
                case NONE:
                    break;

                case ONE:
                    break;

            case FOUR:
                //...
        }
        phoneCam.stopStreaming();
    }
}