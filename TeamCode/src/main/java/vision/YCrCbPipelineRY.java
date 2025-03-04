package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class YCrCbPipelineRY extends OpenCvPipeline {
    private Scalar LOWER_YELLOW = new Scalar(50, 100, 0);
    private Scalar UPPER_YELLOW = new Scalar(255, 255, 100);

    private Scalar LOWER_RED = new Scalar(50, 180, 0);
    private Scalar UPPER_RED = new Scalar(255, 255, 140);

    private int MIN_CONTOUR_AREA = 10000;

    private int X_MIN = 0, X_MAX = 640;  // Adjust as needed
    private int Y_MIN = 0, Y_MAX = 480;  // Adjust as needed

    private Mat yCrCb = new Mat();
    private Mat maskYellow = new Mat();
    private Mat maskRed = new Mat();
    private Mat hierarchy = new Mat();

    private Rect largestBoundingBox = null;
    private boolean blockDetected = false;
    private int width = 0;
    private int height = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert to YCrCb color space
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Threshold for yellow
        Core.inRange(yCrCb, LOWER_YELLOW, UPPER_YELLOW, maskYellow);
        // Threshold for red
        Core.inRange(yCrCb, LOWER_RED, UPPER_RED, maskRed);

        // Find contours
        List<MatOfPoint> yellowContours = new ArrayList<>();
        List<MatOfPoint> redContours = new ArrayList<>();
        Imgproc.findContours(maskYellow, yellowContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskRed, redContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // makes it easier: combine all contours
        yellowContours.removeAll(redContours);
        yellowContours.addAll(redContours);
        largestBoundingBox = findLargestContourInRange(yellowContours);

        if (largestBoundingBox != null) {
            Imgproc.rectangle(input, largestBoundingBox.tl(), largestBoundingBox.br(), new Scalar(0, 255, 0), 2);
            blockDetected = true;
            width = largestBoundingBox.width;
            height = largestBoundingBox.height;
        } else {
            blockDetected = false;
        }

        return input; // Return processed frame
    }

    private Rect findLargestContourInRange(List<MatOfPoint> contours) {
        Rect largestBox = null;
        double maxArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA) {
                Moments M = Imgproc.moments(contour);
                if (M.m00 != 0) {
                    int cx = (int) (M.m10 / M.m00);
                    int cy = (int) (M.m01 / M.m00);

                    // Check if centroid is within the valid range
                    if (cx >= X_MIN && cx <= X_MAX && cy >= Y_MIN && cy <= Y_MAX) {
                        if (area > maxArea) {
                            maxArea = area;
                            largestBox = Imgproc.boundingRect(contour);
                        }
                    }
                }
            }
        }
        return largestBox;
    }

    public void setLOWER_YELLOW(int y, int cr, int cb) {
        LOWER_YELLOW = new Scalar(y, cr, cb);
    }

    public void setUPPER_YELLOW(int y, int cr, int cb) {
        UPPER_YELLOW = new Scalar(y, cr, cb);
    }

    public void setLOWER_RED(int y, int cr, int cb) {
        LOWER_RED = new Scalar(y, cr, cb);
    }

    public void setUPPER_RED(int y, int cr, int cb) {
        UPPER_RED = new Scalar(y, cr, cb);
    }

    public void setX_MIN(int x) {
        X_MIN = x;
    }
    public void setX_MAX(int x) {
        X_MAX = x;
    }

    public void setY_MIN(int y) {
        Y_MIN = y;
    }
    public void setY_MAX(int y) {
        Y_MAX = y;
    }

    public void setMIN_CONTOUR_AREA(int a) {
        MIN_CONTOUR_AREA = a;
    }

    public boolean isBlockDetected() {
        return blockDetected;
    }

    public int getOrientation() {
        if (!blockDetected) {
            return -1; // No block detected
        }

        return (width > height) ? 1 : 0; // 1: horizontal, 0: vertical
    }
}
