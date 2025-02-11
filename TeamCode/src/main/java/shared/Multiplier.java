package shared;

public class Multiplier {
    private double m1 = 0.4;
    private double m2 = 0.3;
    private double m3 = 0.2;
    private double m4 = 0.1;
    private double mLast = 0.2;

    // in
    private double pivotPoint = 2;

    private double ticksPerInch = 29;

    private double l1 = 10;
    private double l2 = 9;
    private double l3 = 8;
    private double l4 = 8;
    private double lLast = 2;

    public double calculateMultiplier(int extendLength) {
        double lengthInches = extendLength / ticksPerInch;
        double total = m1 * (l1/2) +
                m2 * (l1 + Math.min(lengthInches/2, l2/2)) +
                m3 * (l1 + Math.min(lengthInches/2, l2+l3/2)) +
                m4 * (l1 + Math.min(lengthInches/2, l2+l3+l4/2)) +
                mLast * (l1 + Math.min(lengthInches/2, l2+l3+l4+lLast/2));

        return (total / (m1+m2+m3+m4+mLast)) - pivotPoint;
    }
}
