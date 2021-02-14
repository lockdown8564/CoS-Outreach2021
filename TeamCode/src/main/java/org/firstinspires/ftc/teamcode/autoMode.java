package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "autonomous", group = "Auto")
public class autoMode extends LinearOpMode {

    Hardware robot = new Hardware();

    OpenCvCamera webCam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(robot.hwMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        ringStackSizeDetector stackSizeDetector = new ringStackSizeDetector(telemetry);
        webCam.setPipeline(stackSizeDetector);
        webCam.openCameraDeviceAsync(
                () -> webCam.startStreaming(240, 320, OpenCvCameraRotation.UPRIGHT)
        );

        waitForStart();
        //pick up Wobble
        switch (stackSizeDetector.stackSize()) {
            case NONE:
                //...
                break;
            case ONE:
                //...
                break;

            case FOUR:
                //...
        }
        //Drop off Wobble to square
// need team string
// need startposition
// need to place
// need
       /*
       switch(team)

       {

        */
        case RED: {
            switch (startposition) {
                case REDLEFT: {
                    switch (position) {
                        case FOUR: {
                            // code to turn right
                            robot.frontLeftDrive.setPower(1);
                            robot.frontRightDrive.setPower(-1);
                            robot.backLeftDrive.setPower(1);
                            robot.backRightDrive.setPower(-1);
                            toC();

                            break;
                        }
                        case ONE: {
                            toB();
                            break;
                        }
                        case NONE: {|
                            //code to turn right
                            robot.frontLeftDrive.setPower(1);
                            robot.frontRightDrive.setPower(-1);
                            robot.backLeftDrive.setPower(1);
                            robot.backRightDrive.setPower(-1);
                            toA();

                            break;
                        }
                    }
                    break;
                }
                case REDRIGHT: {
                    switch (position) {
                        case FOUR: {
                            toC();
                            break;
                        }
                        case ONE: {
                            //code to turn right
                            robot.frontLeftDrive.setPower(-1);
                            robot.frontRightDrive.setPower(1);
                            robot.backLeftDrive.setPower(-1);
                            robot.backRightDrive.setPower(1);
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
            }


            break;
        }


           /*
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

            */

        //Shoot Rings

        //Park on Line

        webCam.stopStreaming();


        //functions to get the robot to the spot
        public static void toA() {
            //code to move forward until reaching the poin
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            sleep(2000);
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            //stop at the point and place wobble
        }

        public static void toB() {
            //align
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            sleep(3000);
            //stop at the point and place wobble
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);


        }

        public static void toC() {
            //align
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            sleep(4000);
            //stop at the point and place wobble
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);


        }
    }
}