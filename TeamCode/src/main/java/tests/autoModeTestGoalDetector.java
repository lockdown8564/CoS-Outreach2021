package tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
// ^ nvm it's time to redo the whole odo thing to a diff algorithm (roadrunner) bc everyone uses roadrunner lol

@Autonomous(name = "autonomous", group = "Auto")
public class autoModeTestGoalDetector extends LinearOpMode {

    Hardware robot = new Hardware();

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCVGoalDetector detector = new OpenCVGoalDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(
                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        waitForStart();
        //pick up Wobble
        switch (detector.getGoalPresence()) {
            case PRESENT:
                //...
                break;
            case ABSENT:
                //...
        }
        phoneCam.stopStreaming();
    }
}
