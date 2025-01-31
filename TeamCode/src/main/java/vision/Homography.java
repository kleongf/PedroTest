package vision;

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.calib3d.Calib3d;
import java.io.*;

public class Homography {
    private static final double[][] H = new double[3][3];
    // double x_i = 500, y_i = 300; // Change this as needed

    // Convert image coordinates to world coordinates
    // double[] worldCoords = imageToWorld(H, x_i, y_i);
//    double X_w = worldCoords[0];
//    double Y_w = worldCoords[1];

    // Known Z_w
//    double Z_w = 1.5; // Change as needed
//
//    System.out.println("3D Position: (" + X_w + ", " + Y_w + ", " + Z_w + ")");


    public static double[] imageToWorld(double x_i, double y_i) {
        double[] worldPoint = new double[3];

        // Apply homography transformation
        worldPoint[0] = H[0][0] * x_i + H[0][1] * y_i + H[0][2];
        worldPoint[1] = H[1][0] * x_i + H[1][1] * y_i + H[1][2];
        worldPoint[2] = H[2][0] * x_i + H[2][1] * y_i + H[2][2];

        // Normalize by dividing by w
        double X_w = worldPoint[0] / worldPoint[2];
        double Y_w = worldPoint[1] / worldPoint[2];

        return new double[]{X_w, Y_w};
    }
}

