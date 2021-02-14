package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ringStackSizeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum StackSize {
        NONE,
        ONE,
        FOUR
    }

    private StackSize stackSize;

    static final Rect MAIN_REGION_OF_INTEREST = new Rect(
            new Point(70, 75),
            new Point(170,250));
    static double ONE_THRESHOLD = 0.2;
    static double FOUR_THRESHOLD = 0.35;
    public ringStackSizeDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowYellowHSV = new Scalar(23, 50, 70);
        Scalar highYellowHSV = new Scalar(32, 255, 255);


        Core.inRange(mat, lowYellowHSV, highYellowHSV, mat);
        Mat fromMain = mat.submat(MAIN_REGION_OF_INTEREST);

        double mainValue = Core.sumElems(fromMain).val[0] / MAIN_REGION_OF_INTEREST.area() / 255;

        fromMain.release();

        telemetry.addData("Main raw val: ", (int) Core.sumElems(fromMain).val[0]);
        telemetry.addData("mainValue percentage: ", Math.round(mainValue * 100) + "%");

        boolean noneVal = mainValue < ONE_THRESHOLD;
        boolean oneVal = (mainValue >= ONE_THRESHOLD) && (mainValue < FOUR_THRESHOLD);

        if (noneVal) {
            stackSize = StackSize.NONE;
            telemetry.addData("Stack Size ", "Zero");
        } else if (oneVal) {
            stackSize = StackSize.ONE;
            telemetry.addData("Stack Size ", "One");
        } else {
            stackSize = StackSize.FOUR;
            telemetry.addData("Stack Size", "Four");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar noneColor = new Scalar(255, 0, 0);
        Scalar atLeastOneColor = new Scalar (0, 0, 255);

        Imgproc.rectangle(mat, MAIN_REGION_OF_INTEREST, stackSize == StackSize.NONE? noneColor:atLeastOneColor);

        return mat;
    }

    public StackSize stackSize() {
        return stackSize;
    }
}

