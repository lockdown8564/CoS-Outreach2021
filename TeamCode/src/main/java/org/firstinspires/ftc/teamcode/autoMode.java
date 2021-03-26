/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import ftclib.FtcMenu;
import ftclib.FtcChoiceMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@Autonomous(name = "auton", group = "Auto")
public class autoMode extends LinearOpMode implements FtcMenu.MenuButtons {
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0.30, 0.10, 0.20);
    private static HalDashboard dashboard;
    Alliance alliance;
    StartingPos startpos;
    HighGoalFromPreload highGoalFromPreload;
    WobbleGoal wobbleGoal;
    HighGoalAfterWobble highGoalAfterWobble;
    Park park;
    private int delay;
    OpenCvCamera webCam;
    DeterminationPipeline pipeline;
    private int newWobbleArmTargetPos;
    private double integral = 0;

    private enum Alliance {
        RED,
        BLUE
    }

    private enum StartingPos {
        WALL,
        CENTER
    }

    private enum HighGoalFromPreload {
        YES,
        NO
    }

    private enum WobbleGoal {
        YES,
        NO
    }

    private enum HighGoalAfterWobble {
        YES,
        NO
    }

    private enum Park {
        YES,
        NO
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        alliance = Alliance.RED;
        startpos = StartingPos.WALL;
        highGoalFromPreload = HighGoalFromPreload.YES;
        wobbleGoal = WobbleGoal.YES;
        highGoalAfterWobble = HighGoalAfterWobble.NO;
        park = Park.YES;
        delay = 0;

        dashboard = HalDashboard.createInstance(telemetry);
        robot.shooterServo.setPosition(1);

        doMenus();

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                        cameraMonitorViewId);
        pipeline = new DeterminationPipeline();
        webCam.setPipeline(pipeline);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(1920, 1080, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

//      if (position == DeterminationPipeline.RingPosition.NONE) {
//              toA();
//      } else if (position == DeterminationPipeline.RingPosition.ONE) {
//              toB();
//      } else if (position == DeterminationPipeline.RingPosition.FOUR) {
//              toC();
//      }
        waitForStart();

        if (pipeline.position == DeterminationPipeline.RingPosition.NOT_DETECTED) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } else {
                sleep(1000);
                webCam.stopStreaming();
        }
        if(opModeIsActive()) {
            switch (alliance) {
                case RED: {
                    switch (startpos) {
                        case WALL: {
                            switch (highGoalFromPreload) {
                                case YES: {
                                    PidDrive( -6);
                                    robot.moveArm(0);
                                    sleep(1500);
                                    // tests
//                                    encImuDrive(0.05, -58, 0);
//                                    turnLeftEnc(120, 0.5);
//                                    sleep(50);

                                    shootRings();
//                                    sleep(5000);
//                                    runtime.reset();
//                                    sleep(50);
//                                    PidDrive(0.1, 7);
//                                    runtime.reset();
//                                    sleep(50);
//                                    if (pipeline.position == DeterminationPipeline.RingPosition.NONE) {
//                                        toA();
//                                    } else if (pipeline.position == DeterminationPipeline.RingPosition.ONE) {
//                                        toB();
//                                    } else if (pipeline.position == DeterminationPipeline.RingPosition.FOUR) {
//                                        toC();
//                                    }
                                    switch (wobbleGoal) {
                                        case YES: {
                                            switch (pipeline.position) {
                                                case NONE: {
                                                    toA();
                                                    switch (park) {
                                                        case YES: {
//                                                        park();
                                                            break;
                                                    }
                                                        case NO: {
                                                    }
                                                }
                                                break;
                                            }
                                            case ONE: {
                                                toB();
                                                break;
                                            }
                                            case FOUR: {
                                                toC();
                                            }
                                        }
                                        break;
                                    }
                                    case NO: {
                                        switch (park) {
                                            case YES: {
                                                break;
                                            }
                                            case NO: {

                                            }
                                        }
                                        break;
                                    }
                                }
                                break;
                            }
                            case NO: {
                                break;
                            }
                        }
                        switch (highGoalAfterWobble) {
                            case NO:
                                break;
                            case YES:
                                //......
                        }
                    }
                    case CENTER: {
                        break;
                    }
                }
                break;
            }
            case BLUE: {
                break;
            }
        }
        }


//        robot.setAllDrivePower(1);
//        sleep(1800);
//        robot.setAllDrivePower(0);
//        sleep(500);
//        robot.wobbleArm.setPower(-1);
//        sleep(4000);
//        robot.wobbleArm.setPower(0);


// to-do: create functions for parking, grabbing wobble goal, & robot shooter stuff
//        while (opModeIsActive()) {
//        }
    }
// to-do: make a function that grabs the wobble goal
    private void toA() {
        turnLeftEnc(85, 1);
        robot.moveArm(-1);
        sleep(3000);
        robot.wobbleServo.setPosition(robot.SERVO_RELEASE);
        sleep(700);
        robot.moveArm(1);
        sleep(3000);
        encoderDrive(1, -10, -10, 2);
        turnLeftEnc(95, 1);
        sleep(2000);
        encoderDrive(1, 20, -20, 2);
//        turnRightEnc(90, 0.3);
//        // grab second wobble
//        turnLeftEnc(90, 0.3);
//        encImuDrive(0.1, -70, 0);
//        // let go of first wobble here
    }

    private void toB() {
        turnLeftEnc(280, 0.8);
        robot.moveArm(-1);
        sleep(700);
        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        sleep(500);
        turnRightEnc(45, 0.8);
        encoderDrive(1, 72, 72, 5);
        robot.wobbleServo.setPosition(robot.SERVO_RELEASE);
        encoderDrive(1, -20, -20, 3);
        turnRightEnc(120, 0.8);
        encoderDrive(1, 75, 75, 5);
        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        sleep(500);
        turnRightEnc(150, 0.8);
        encoderDrive(1, 72, 72, 5);
        robot.moveArm(1);
        sleep(700);
        turnLeftCurvy(100, 1);
        encoderDrive(1, 20, 20, 4);
        turnRightCurvy(90, 1);
    }

    private void toC() {
        turnLeftEnc(280, 0.8);
        robot.moveArm(-1);
        sleep(700);
        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        sleep(500);
        turnRightEnc(45, 0.8);
        encoderDrive(1, 96, 96, 5);
        robot.wobbleServo.setPosition(robot.SERVO_RELEASE);
        encoderDrive(1, -20, -20, 3);
        turnRightEnc(120, 0.8);
        encoderDrive(1, 100, 100, 5);
        robot.wobbleServo.setPosition(robot.SERVO_GRAB);
        sleep(500);
        turnRightEnc(150, 0.8);
        encoderDrive(1, 96, 96, 5);
        robot.moveArm(1);
        sleep(700);
        turnLeftCurvy(100, 1);
        encoderDrive(1, 36, 36, 4);
        turnRightCurvy(90, 1);
    }

    private void shootRings() {
            robot.flywheel.setPower(1);
            robot.flywheel2.setPower(-1);
            sleep(2000);
            robot.shooterServo.setPosition(0.3);
            sleep(1000);
            robot.shooterServo.setPosition(0.85);
            sleep(1000);
            robot.shooterServo.setPosition(0.3);
            sleep(1000);
            robot.shooterServo.setPosition(0.85);
            sleep(1000);
            robot.shooterServo.setPosition(0.3);
            sleep(800);
            robot.flywheel.setPower(0);
            robot.flywheel2.setPower(0);
    }

    private void park() {
        encoderDrive(1, 20, 20, 4);
    }

    public void PidDrive(double distance) {
        double repCount = 0;
        if(opModeIsActive()) {
        double targetPos = (int)(distance*robot.getCountsPerInch()) + (((robot.backRightDrive.getCurrentPosition()
                + robot.backLeftDrive.getCurrentPosition() + robot.frontRightDrive.getCurrentPosition()
                + robot.frontLeftDrive.getCurrentPosition()) / 4));
            double error = ((robot.backRightDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition()
                    + robot.frontRightDrive.getCurrentPosition() + robot.frontLeftDrive.getCurrentPosition()) / 4);
            double lastError = 0;
            while (Math.abs(error) <= 9 && opModeIsActive() && Math.abs(repCount) < Math.abs(targetPos)) {
                error = robot.backRightDrive.getCurrentPosition() - targetPos;
                double changeInError = lastError - error;
                integral += changeInError * runtime.time();
                double derivative = changeInError / runtime.time();
                double P = pidCoeffs.p * error;
                double I = pidCoeffs.i * integral;
                double D = pidCoeffs.d * derivative;
                robot.setAllDrivePower(P + I + D);
                error = lastError;
                repCount++;
                runtime.reset();
            }
            robot.stopAllMotorPower();
            error = ((robot.backRightDrive.getCurrentPosition() + robot.backLeftDrive.getCurrentPosition()
                    + robot.frontRightDrive.getCurrentPosition() + robot.frontLeftDrive.getCurrentPosition()) / 4);
            repCount = 0;
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encoderDrive(double speed, double lInches, double rInches, int time){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(lInches*robot.getCountsPerInch()) + robot.frontLeftDrive.getCurrentPosition();
            RIGHT_TARGET = (int)(rInches*robot.getCountsPerInch()) + robot.frontRightDrive.getCurrentPosition();

            robot.verticalDriveSetTarget(LEFT_TARGET, RIGHT_TARGET);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.setAllDrivePower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < time) && robot.driveIsBusy()) {
                telemetry.addData("path:", "running to %7d :%7d", LEFT_TARGET, RIGHT_TARGET);
                telemetry.addData("path:", "running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(), robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    private void encoderDriveSide(double speed, double lInches, double rInches, int direction, int time){
        int LEFT_TARGET, RIGHT_TARGET;
        if(opModeIsActive()){
            LEFT_TARGET = (int)(lInches*robot.getCountsPerInch()) + robot.frontLeftDrive.getCurrentPosition();
            RIGHT_TARGET = (int)(rInches*robot.getCountsPerInch()) + robot.frontRightDrive.getCurrentPosition();

            robot.sideDriveSetTarget(direction, LEFT_TARGET, RIGHT_TARGET);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.setAllDrivePower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < time) && robot.driveIsBusy()) {
                telemetry.addData("path:", "running to %7d :%7d", LEFT_TARGET, RIGHT_TARGET);
                telemetry.addData("path:", "running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(), robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void turnRightEnc(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle<=TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, -power, power, -power);
                telemetry.addData("Heading:", currentAngle);
                telemetry.update();
            }
            robot.stopAllMotorPower();
            break;
        }
    }

    private void turnLeftEnc(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(-power, power, -power, power);
            }
            robot.stopAllMotorPower();
            break;
        }
    }

    private void turnLeftCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power*4, power, power*4, power);
            }
            robot.stopAllMotorPower();
            break;
        }
    }

    private void turnRightCurvy(final float TARGET_ANGLE, double power){
        while(opModeIsActive()){
            float currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
            while(currentAngle>=-TARGET_ANGLE){
                currentAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle;
                robot.driveSetPower(power, power*6, power, power*6);
            }
            robot.stopAllMotorPower();
            break;
        }
    }

    private void encImuDrive(double speed, double distance, double angle){
        int LEFT_TARGET, RIGHT_TARGET;
        double error;
        double steer;
        double leftSpeed, rightSpeed;
        double max;

        if(opModeIsActive()){
            LEFT_TARGET = (int)(distance*robot.getCountsPerInch()) + robot.frontLeftDrive.getCurrentPosition();
            RIGHT_TARGET = (int)(distance*robot.getCountsPerInch()) + robot.frontRightDrive.getCurrentPosition();

            robot.verticalDriveSetTarget(LEFT_TARGET,RIGHT_TARGET);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setAllDrivePower(Math.abs(speed));

            while(robot.driveIsBusy() && opModeIsActive()){
                error = robot.getErrorForPID(angle);
                steer = robot.getSteer(error, robot.P_DRIVE_COEFF);

                if (distance < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

                if(max > 1.0){
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.driveSetPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);

            }

            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encStrafe(double speed, double distance){
        int flTarget, frTarget, blTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeftDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());
            frTarget = (robot.frontRightDrive.getCurrentPosition()) - (int)(distance*robot.getCountsPerInch());
            blTarget = (robot.backLeftDrive.getCurrentPosition()) - (int)(distance*robot.getCountsPerInch());
            brTarget = (robot.backRightDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());

            robot.strafeDriveSetTarget(flTarget, frTarget, blTarget, brTarget);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setAllDrivePower(Math.abs(speed));

            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void encStrafeFlBr(double speed, double distance){
        int flTarget, brTarget;

        if(opModeIsActive()){
            flTarget = (robot.frontLeftDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());
            brTarget = (robot.backRightDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());

            robot.frontLeftDrive.setTargetPosition(flTarget);
            robot.backRightDrive.setTargetPosition(brTarget);

            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(Math.abs(speed), 0, 0 ,Math.abs(speed));

            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // to do: sort out enc drive code and fix whatever is going on with these methods lol
    private void encStrafeFrBl(double speed, double distance){
        int frTarget, blTarget;

        if(opModeIsActive()){
            frTarget = (robot.frontLeftDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());
            blTarget = (robot.backRightDrive.getCurrentPosition()) + (int)(distance*robot.getCountsPerInch());

            robot.frontRightDrive.setTargetPosition(frTarget);
            robot.backLeftDrive.setTargetPosition(blTarget);

            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.driveSetPower(0, Math.abs(speed), Math.abs(speed), 0);


            robot.stopAllMotorPower();
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    // FTC Menu Implementation
    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus(){
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null, this);
        FtcChoiceMenu<StartingPos> startPosMenu = new FtcChoiceMenu<>("Start Position:", allianceMenu, this);
        FtcChoiceMenu<HighGoalFromPreload> highGoalFromPreloadMenu = new FtcChoiceMenu<>("High Goal At Start:", startPosMenu, this);
        FtcChoiceMenu<WobbleGoal> wobbleGoalMenu = new FtcChoiceMenu<>("Wobble Goal:", highGoalFromPreloadMenu, this);
        FtcChoiceMenu<HighGoalAfterWobble> highGoalAfterWobbleMenu = new FtcChoiceMenu<>("Score Rings After Wobble:", wobbleGoalMenu, this);
        FtcChoiceMenu<Park> parkMenu = new FtcChoiceMenu<>("Park:", highGoalAfterWobbleMenu, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", parkMenu, this, 0, 25000, 500, 0, "%.0f msec");

        allianceMenu.addChoice("Red", Alliance.RED, true, startPosMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE, false, startPosMenu);

        startPosMenu.addChoice("Wall", StartingPos.WALL, true, highGoalFromPreloadMenu);
        startPosMenu.addChoice("Center", StartingPos.CENTER, false, highGoalFromPreloadMenu);

        highGoalFromPreloadMenu.addChoice("Yes", HighGoalFromPreload.YES, true, wobbleGoalMenu);
        highGoalFromPreloadMenu.addChoice("No", HighGoalFromPreload.NO, false, wobbleGoalMenu);

        wobbleGoalMenu.addChoice("Yes", WobbleGoal.YES, true, highGoalAfterWobbleMenu);
        wobbleGoalMenu.addChoice("No", WobbleGoal.NO, false, highGoalAfterWobbleMenu);

        highGoalAfterWobbleMenu.addChoice("Yes", HighGoalAfterWobble.YES, false, parkMenu);
        highGoalAfterWobbleMenu.addChoice("No", HighGoalAfterWobble.NO, true, parkMenu);

        parkMenu.addChoice("Yes", Park.YES, true, delayMenu);
        parkMenu.addChoice("No", Park.NO, false, delayMenu);

        delayMenu.setChildMenu(null);

        FtcMenu.walkMenuTree(allianceMenu, this);
        alliance = allianceMenu.getCurrentChoiceObject();
        startpos = startPosMenu.getCurrentChoiceObject();
        highGoalFromPreload = highGoalFromPreloadMenu.getCurrentChoiceObject();
        wobbleGoal = wobbleGoalMenu.getCurrentChoiceObject();
        highGoalAfterWobble = highGoalAfterWobbleMenu.getCurrentChoiceObject();
        park = parkMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();

        dashboard.displayPrintf(0, "Alliance: %s (%s)", allianceMenu.getCurrentChoiceText(), alliance.toString());
        dashboard.displayPrintf(1, "Start Position: %s (%s)", startPosMenu.getCurrentChoiceText(), startpos.toString());
        dashboard.displayPrintf(2, "High Goal From Preload: %s (%s)", highGoalFromPreloadMenu.getCurrentChoiceText(), highGoalFromPreload.toString());
        dashboard.displayPrintf(3, "Score Wobble Goal: %s (%s)", wobbleGoalMenu.getCurrentChoiceText(), wobbleGoal.toString());
        dashboard.displayPrintf(4, "High Goal After Wobble: %s (%s)", highGoalAfterWobbleMenu.getCurrentChoiceText(), highGoalAfterWobble.toString());
        dashboard.displayPrintf(5, "Park: %s (%s)", parkMenu.getCurrentChoiceText(), park.toString());
        dashboard.displayPrintf(6, "Delay: %d msec", delay);
    }
// End of FTC Menu



// Ring Stack Size Detector Code
    public static class DeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE,
            NOT_DETECTED
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         * NEEDS TO CHANGE
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(200,500);

        static final int REGION_WIDTH = 500;
        static final int REGION_HEIGHT = 300;

        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile RingPosition position;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.NOT_DETECTED;
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
// End of Ring Stack Size Detector Code

}