package vision;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvPipeline;

public class YellowPipeline extends OpenCvPipeline {
    // Adjusted HSV range for better yellow detection
    private static final Scalar LOWER_YELLOW = new Scalar(20, 170, 170);  // Less sensitive to reds and blues
    private static final Scalar UPPER_YELLOW = new Scalar(40, 255, 255);  // More strictly within yellow range

    // ROI bounds
    private static final int ROI_X_START = 200;
    private static final int ROI_X_END = 440;
    private static final int ROI_Y_START = 140;
    private static final int ROI_Y_END = 340;

    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();

    private boolean yellowPixelDetected = false;

    @Override
    public Mat processFrame(Mat input) {
        // Reset detection flag
        yellowPixelDetected = false;

        // Convert frame to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold image to get only yellow colors (more strict range)
        Core.inRange(hsvMat, LOWER_YELLOW, UPPER_YELLOW, thresholdMat);

        // Extract region of interest (ROI) from thresholded image
        Rect roi = new Rect(ROI_X_START, ROI_Y_START, ROI_X_END - ROI_X_START, ROI_Y_END - ROI_Y_START);
        Mat roiMat = thresholdMat.submat(roi);

        // Check if any yellow pixels are detected in the ROI
        if (Core.countNonZero(roiMat) > 100) {
            yellowPixelDetected = true;

            // Draw a rectangle around the ROI on the input frame
            Imgproc.rectangle(input, roi.tl(), roi.br(), new Scalar(0, 255, 255), 2);
        }

        // Return the processed frame
        return input;
    }

    /**
     * Returns true if at least one yellow pixel is detected within the ROI.
     */
    public boolean isBlockDetected() {
        return yellowPixelDetected;
    }
}
