package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowBluePipeline extends OpenCvPipeline {

    // Define HSV color ranges for red and yellow
    private static final Scalar LOWER_RED1 = new Scalar(0, 100, 100);
    private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED2 = new Scalar(170, 100, 100);
    private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);
    private static final Scalar LOWER_YELLOW = new Scalar(11, 100, 100);
    private static final Scalar UPPER_YELLOW = new Scalar(40, 255, 255);
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 100);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    // Define Region of Interest (ROI) coordinates
    private static final int ROI_X_START = 0;
    private static final int ROI_X_END = 200;
    private static final int ROI_Y_START = 140;
    private static final int ROI_Y_END = 340;

    // Mats for processing images
    private Mat hsvMat = new Mat();
    private Mat blueMask = new Mat();
    private Mat yellowMask = new Mat();
    private boolean blockDetected = false;
    private int width = 0;
    private int height = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert input image from BGR to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);


        Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, blueMask);

        Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, yellowMask);

        // Define the Region of Interest (ROI)
        Rect roi = new Rect(ROI_X_START, ROI_Y_START, ROI_X_END - ROI_X_START, ROI_Y_END - ROI_Y_START);
        Mat blueROI = blueMask.submat(roi);
        Mat yellowROI = yellowMask.submat(roi);

        // Find contours in the red and yellow regions
        java.util.List<MatOfPoint> blueContours = new java.util.ArrayList<>();
        java.util.List<MatOfPoint> yellowContours = new java.util.ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(blueROI, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(yellowROI, yellowContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Initialize variables for the largest contour
        double maxArea = 0;
        MatOfPoint largestContour = null;

        // Check red contours
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

        double areaRatio = 0;
        if (largestContour != null) {
            areaRatio = (Imgproc.contourArea(largestContour)/(roi.width * roi.height));
        }

        // If a largest contour was found, draw it on the frame
        if (largestContour != null && areaRatio > 0.3) {
            blockDetected = true;
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            Scalar color = (blueContours.contains(largestContour)) ? new Scalar(0, 0, 255) : new Scalar(0, 255, 255);  // Red for red contours, yellow for yellow

            // Draw bounding box around the largest contour
            Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), color, 2);
            width = boundingRect.width;
            height = boundingRect.height;
        } else {
            blockDetected = false;
        }

        // Return the processed frame
        Imgproc.rectangle(input, roi.tl(), roi.br(), new Scalar(0, 255, 255), 2);
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
        // i swapped these bcause camera is sideways
        return (width*1.4 > height) ? 0 : 1; // 1: Horizontal, 0: Vertical
    }
}