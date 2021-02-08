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
public class autoMode extends LinearOpMode {

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
                            robot. frontRightDrive.setPower(-1);
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
                            robot. backLeftDrive.setPower(-1);
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

        phoneCam.stopStreaming();


        //functions to get the robot to the spot
        public static void toA() {
            //code to move forward until reaching the poin
            robot.frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            robot.sleep(2000);
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
            //stop at the point and place wobble
        }

        public static void toB() {
            //align
            robot. frontLeftDrive.setPower(1);
            robot.frontRightDrive.setPower(-1);
            robot.backLeftDrive.setPower(1);
            robot.backRightDrive.setPower(-1);
            robot.sleep(3000);
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
            robot.sleep(4000);
            //stop at the point and place wobble
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);


        }
    }