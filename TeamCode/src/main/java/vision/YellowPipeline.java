package vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;

import java.util.ArrayList;
import java.util.List;

public class YellowPipeline extends OpenCvPipeline {
    Scalar LOWER_YELLOW = new Scalar(10, 100, 100);
    Scalar UPPER_YELLOW = new Scalar(40, 255, 255);

    double[][] homographyMatrix = {
            {1.2, 0.0, -100},
            {0.0, 1.2, -50},
            {0.0, 0.0, 1.0}
    };

    private double[] applyHomography(double centerX, double centerY) {
        double worldX = (homographyMatrix[0][0] * centerX + homographyMatrix[0][1] * centerY + homographyMatrix[0][2]) /
                (homographyMatrix[2][0] * centerX + homographyMatrix[2][1] * centerY + homographyMatrix[2][2]);
        double worldY = (homographyMatrix[1][0] * centerX + homographyMatrix[1][1] * centerY + homographyMatrix[1][2]) /
                (homographyMatrix[2][0] * centerX + homographyMatrix[2][1] * centerY + homographyMatrix[2][2]);
        return new double[]{worldX, worldY};
    }

    private double[] closestContour = new double[2];

    @Override
    public Mat processFrame(Mat input) {
        Mat imgHSV = new Mat();
        Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_RGB2HSV);

        Mat imgThreshold = new Mat();
        Core.inRange(imgHSV, LOWER_YELLOW, UPPER_YELLOW, imgThreshold);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(imgThreshold, imgThreshold, Imgproc.MORPH_OPEN, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(imgThreshold, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = Double.MAX_VALUE;

        if (contours.isEmpty()) {
            closestContour[0] = -100;
            closestContour[1] = -100;
        } else {
            for (MatOfPoint contour : contours) {
                if (Imgproc.contourArea(contour) > 500) {
                    Rect rect = Imgproc.boundingRect(contour);
                    if (rect.width > rect.height) {
                        Point center = new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
                        double[] worldCoords = applyHomography(center.x, center.y);
                        double distance = Math.hypot(worldCoords[0], worldCoords[1]);
                        if (distance < minDistance) {
                            minDistance = distance;
                            closestContour[0] = worldCoords[0];
                            closestContour[1] = worldCoords[1];
                        }
                    }
                }
            }
        }

        telemetry.addData("Closest World X", closestContour[0]);
        telemetry.addData("Closest World Y", closestContour[1]);
        return input;
    }

    public double[] getClosest() {
        return closestContour;
    }
}
