package tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class StackSizeDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    enum StackSize {
        NONE,
        ONE,
        FOUR
    }

    private StackSize stackSize;

    static final Rect MAIN_ROI = new Rect(
            new Point(70, 75),
            new Point(170, 250));

    static double ONE_RING_THRESHOLD = 0.2;
    static double FOUR_RING_THRESHOLD = 0.35;
    public StackSizeDetector (Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowYellowHSV = new Scalar(23, 50, 70);
        Scalar highYellowHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowYellowHSV, highYellowHSV, mat);
        Mat mainMat = mat.submat(MAIN_ROI);

        double mainVal = Core.sumElems(mainMat).val[0] / MAIN_ROI.area() / 255;
        mainMat.release();

        telemetry.addData("Main raw val", (int) Core.sumElems(mainMat).val[0]);
        telemetry.addData("Main percentage", Math.round(mainVal * 100) + "%");

        boolean oneVal = (mainVal >= ONE_RING_THRESHOLD) && (mainVal < FOUR_RING_THRESHOLD);
        boolean fourVal = mainVal >= FOUR_RING_THRESHOLD;

        if (oneVal) {
            stackSize = StackSize.ONE;
            telemetry.addData("stack size", "1");
        } else if (fourVal) {
            stackSize = StackSize.FOUR;
            telemetry.addData("stack size", "4");
        } else {
            stackSize = StackSize.NONE;
            telemetry.addData("stack size", "0");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar noneColor = new Scalar(255, 0, 0);
        Scalar atLeastOneColor = new Scalar(0, 0, 255);

        Imgproc.rectangle(mat, MAIN_ROI, stackSize == StackSize.NONE? noneColor:atLeastOneColor);

        return mat;
    }

    public StackSize getStackSize() {
        return stackSize;
    }

}
