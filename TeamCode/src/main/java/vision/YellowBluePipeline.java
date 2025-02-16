package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowBluePipeline extends OpenCvPipeline {

    // Define HSV color ranges for red, blue, and yellow
    private static final Scalar LOWER_RED1 = new Scalar(0, 100, 100);
    private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED2 = new Scalar(170, 100, 100);
    private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 100);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);
    private static final Scalar LOWER_YELLOW = new Scalar(20, 100, 100);
    private static final Scalar UPPER_YELLOW = new Scalar(30, 255, 255);

    // Define Region of Interest (ROI) coordinates
    private static final int ROI_X_START = 200;
    private static final int ROI_X_END = 440;
    private static final int ROI_Y_START = 80;
    private static final int ROI_Y_END = 280;

    // Mats for processing images
    private Mat hsvMat = new Mat();
    private Mat redMask = new Mat();
    private Mat blueMask = new Mat();
    private Mat yellowMask = new Mat();
    private Mat roiMat;
    private boolean blockDetected = false;
    private int width = 0;
    private int height = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert input image from BGR to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);

        // Create binary masks for detecting red, blue, and yellow colors
        Core.inRange(hsvMat, LOWER_RED1, UPPER_RED1, redMask);
        Mat tempRedMask = new Mat();
        Core.inRange(hsvMat, LOWER_RED2, UPPER_RED2, tempRedMask);
        Core.bitwise_or(redMask, tempRedMask, redMask); // Combine red masks

        Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, blueMask);
        Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        // Define the Region of Interest (ROI)
        Rect roi = new Rect(ROI_X_START, ROI_Y_START, ROI_X_END - ROI_X_START, ROI_Y_END - ROI_Y_START);
        Mat blueROI = blueMask.submat(roi);
        Mat yellowROI = yellowMask.submat(roi);

        // Find contours in the blue and yellow regions
        java.util.List<MatOfPoint> blueContours = new java.util.ArrayList<>();
        java.util.List<MatOfPoint> yellowContours = new java.util.ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(blueROI, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowROI, yellowContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Initialize variables for the largest contour
        double maxArea = 0;
        MatOfPoint largestContour = null;

        // Check blue contours
        for (MatOfPoint contour : blueContours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // Check yellow contours
        for (MatOfPoint contour : yellowContours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        // If a largest contour was found, draw it on the frame
        if (largestContour != null) {
            blockDetected = true;
            width = largestContour.width();
            height = largestContour.height();
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            Scalar color = (blueContours.contains(largestContour)) ? new Scalar(255, 0, 0) : new Scalar(0, 255, 255);  // Blue for blue contours, yellow for yellow

            // Draw bounding box around the largest contour
            Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), color, 2);
        } else {
            blockDetected = false;
        }

        // Return the processed frame
        return input;
    }

    // Method to check if a block is detected
    public boolean isBlockDetected() {
        return blockDetected;
    }

    // Method to determine the orientation of the detected block
    public int getOrientation() {
        if (!blockDetected) {
            return -1; // No block detected
        }
        return (width > height) ? 1 : 0; // 1: Horizontal, 0: Vertical
    }
}

