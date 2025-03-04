package tests;

import java.util.ArrayList;

public class LinearRegression {
    public static double[] computeBestFitLine(ArrayList<double[]> points) {
        int n = points.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;

        for (double[] point : points) {
            double x = point[0];
            double y = point[1];
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        // Calculate slope (a) and intercept (b)
        double a = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        double b = (sumY - a * sumX) / n;

        return new double[]{a, b};
    }
}


