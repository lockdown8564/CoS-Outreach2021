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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
    private Hardware robot = new Hardware();
    private static HalDashboard dashboard;
    Alliance alliance;
    StartingPos startpos;
    HighGoalFromPreload highGoalFromPreload;
    WobbleGoal wobbleGoal;
    HighGoalAfterWobble highGoalAfterWobble;
    Park park;
    int delay;
    OpenCvCamera webCam;
    DeterminationPipeline pipeline;

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

        alliance = Alliance.RED;
        startpos = StartingPos.WALL;
        highGoalFromPreload = HighGoalFromPreload.YES;
        wobbleGoal = WobbleGoal.YES;
        highGoalAfterWobble = HighGoalAfterWobble.NO;
        park = Park.YES;
        delay = 0;

        dashboard = HalDashboard.createInstance(telemetry);
        doMenus();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DeterminationPipeline();
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
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



/* Ring Stack Size Detector Code
 * This class is heavily inspired by FTC Team Wizards.exe #9794.
 * Link to their OpenCV tutorial: https://www.youtube.com/watch?v=-QFoOCoaW7I&t=294s
 */
    public static class DeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
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
        public volatile RingPosition position = RingPosition.FOUR;

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

            position = RingPosition.FOUR; // Record our analysis
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