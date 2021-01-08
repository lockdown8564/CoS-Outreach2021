package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.odo.GlobalCoordinatePositionUpdateSample;
// ^ nvm it's time to redo the whole odo thing to a diff algorithm (roadrunner) bc everyone uses roadrunner lol

@Autonomous(name = "autonomous", group = "Auto")
public class autonomous extends LinearOpMode {
    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCVGoalDetector detector = new OpenCVGoalDetector(telemetry);
        phoneCam.setPipeline(detector);
        phoneCam.openCameraDeviceAsync(
                () -> phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );

        waitForStart();
        switch (detector.getGoalPresence()) {
            case PRESENT:
                //...
                break;
            case ABSENT:
                //...
        }

        phoneCam.stopStreaming();
    }


    /**
     * our alliance for each match
     */
    private enum Alliance {
        /**
         * blue alliance side
         */
        BLUE,

        /**
         * red alliance side
         */
        RED
    }

    private enum StartingPos {
        /**
         * foundation side one tile away from the side wall
         */
        REDLEFT,

        /**
         * stone side along the side wall
         */
        REDRIGHT,

        /**
         * stone side one tile away from the side wall
         */
        BLUELEFT,

        BLUERIGHT
    }

    private enum RingPosition {
        FOUR,
        ONE,
        NONE
    }
    // need team string
    // need startposition
    // need to place
    // need
    switch(team)

    {
        case RED: {
            switch (startposition) {
                case REDLEFT: {
                    switch (position) {
                        case FOUR: {
                            toC();
                            break;
                        }
                        case ONE: {
                            toB();
                            break;
                        }
                        case NONE: {
                            toA();
                            break;
                        }
                    }
                    break;
                }
                case REDRIGHT: {

                }

                break;
            }

        }
        case BLUE: {
            switch (startposition) {
                case BLUELEFT: {
                    break;
                }
                case BLUERIGHT: {
                    break;
                }

            }

        }

    }


    //functions to get the robot to the spot
    public static void toA() {
        //align code here
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);
        sleep();
        //place wobble code here
    }

    public static void toB() {
        //align
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);
        sleep();
        //place wobble code here


    }

    public static void toC() {
        //align
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);
        sleep();
        //place wobble code here

    }
}
