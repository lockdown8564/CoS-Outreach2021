package tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// to-do: modify HSV ranges (see lowHSV and highHSV)
public class OpenCVGoalDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum GoalPresence {
        PRESENT,
        ABSENT
    }
    private GoalPresence goalPresence;

    static final Rect MAIN_ROI = new Rect(
            new Point(70, 75),
            new Point(170,250));
    static double PERCENT_COLOR_THRESHOLD = 0.2;
    public OpenCVGoalDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(0, 100, 100);
        Scalar highHSV = new Scalar(179, 255, 255);


        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat fromMain = mat.submat(MAIN_ROI);

        double mainValue = Core.sumElems(fromMain).val[0] / MAIN_ROI.area() / 255;

        fromMain.release();

        telemetry.addData("Main raw val", (int) Core.sumElems(fromMain).val[0]);
        telemetry.addData("Main percentage", Math.round(mainValue * 100) + "%");

        boolean present = mainValue > PERCENT_COLOR_THRESHOLD;

        if (present) {
            goalPresence = GoalPresence.PRESENT;
            telemetry.addData("Goal Presence: ", "Present");
        } else {
            goalPresence = GoalPresence.ABSENT;
            telemetry.addData("Goal Presence: ", "Absent");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar presentColor = new Scalar(0, 255, 0);
        Scalar absentColor = new Scalar (255, 0, 0);

        Imgproc.rectangle(mat, MAIN_ROI, goalPresence == GoalPresence.PRESENT? presentColor:absentColor);

        return mat;
    }

    public GoalPresence getGoalPresence() {
        return goalPresence;
    }
}

