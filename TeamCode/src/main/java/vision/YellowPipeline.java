package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowPipeline extends OpenCvPipeline {
    // Define HSV color ranges for yellow, red, and blue
    private static final Scalar LOWER_YELLOW = new Scalar(20, 170, 170);
    private static final Scalar UPPER_YELLOW = new Scalar(40, 255, 255);
    private static final Scalar LOWER_RED1 = new Scalar(0, 100, 100);
    private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    private static final Scalar LOWER_RED2 = new Scalar(170, 100, 100);
    private static final Scalar UPPER_RED2 = new Scalar(180, 255, 255);
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 100);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    // Define Region of Interest (ROI) coordinates
    private static final int ROI_X_START = 200;
    private static final int ROI_X_END = 440;
    private static final int ROI_Y_START = 80;
    private static final int ROI_Y_END = 280;

    // Mats for processing images
    private Mat hsvMat = new Mat();
    private Mat yellowMask = new Mat();
    private Mat redMask = new Mat();
    private Mat blueMask = new Mat();
    private Mat roiMat;

    // Boolean to indicate whether a yellow block is detected
    private boolean yellowPixelDetected = false;

    // Variables to store bounding box dimensions
    private int width = 0;
    private int height = 0;

    @Override
    public Mat processFrame(Mat input) {
        yellowPixelDetected = false; // Reset detection flag each frame

        // Convert input image from RGB to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create binary masks for detecting yellow and red colors
        Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, yellowMask);
        Core.inRange(hsvMat, LOWER_RED1, UPPER_RED1, redMask);

        // Combine two red masks to detect red color across both hue ranges
        Mat tempRedMask = new Mat();
        Core.inRange(hsvMat, LOWER_RED2, UPPER_RED2, tempRedMask);
        Core.bitwise_or(redMask, tempRedMask, redMask);

        // Create a binary mask for detecting blue
        Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, blueMask);

        // Define the Region of Interest (ROI)
        Rect roi = new Rect(ROI_X_START, ROI_Y_START, ROI_X_END - ROI_X_START, ROI_Y_END - ROI_Y_START);
        Mat yellowROI = yellowMask.submat(roi);
        Mat redROI = redMask.submat(roi);
        Mat blueROI = blueMask.submat(roi);

        // Count the number of non-zero pixels (detected pixels) in the ROI
        int yellowCount = Core.countNonZero(yellowROI);
        int redCount = Core.countNonZero(redROI);
        int blueCount = Core.countNonZero(blueROI);

        // Calculate the area of the ROI
        int roiArea = roi.width * roi.height;

        // Compute interference ratio (amount of red and blue noise in the ROI)
        double interferenceRatio = (redCount + blueCount) / (double) roiArea;

        // If either yellow or red is detected significantly and interference is low
        if ((yellowCount > 1000 || redCount > 1000) && interferenceRatio < 0.25) {
            yellowPixelDetected = true; // Set detection flag to true
            Imgproc.rectangle(input, roi.tl(), roi.br(), new Scalar(0, 255, 255), 2); // Draw yellow ROI box

            Mat detectedMask = (yellowCount > redCount) ? yellowROI : redROI;

            // Find contours in the detected mask
            java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(detectedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour (most prominent block)
            double maxArea = 0;
            Rect largestRect = new Rect();
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = rect.width * rect.height;
                if (area > maxArea) {
                    maxArea = area;
                    largestRect = rect;
                }
            }

            // Draw a bounding box around the detected largest block
            Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), (yellowCount > redCount) ? new Scalar(0, 255, 255) : new Scalar(0, 0, 255), 2);

            // Store bounding box dimensions for orientation check
            width = largestRect.width;
            height = largestRect.height;
        }

        return input; // Return the processed frame
    }

    // Method to check if a block is detected
    public boolean isBlockDetected() {
        return yellowPixelDetected;
    }

    // Method to determine the orientation of the detected block
    public int getOrientation() {
        if (!yellowPixelDetected) {
            return -1; // No block detected
        }
        return (width > height) ? 1 : 0; // 1: Horizontal, 0: Vertical
    }
}
