package shared;

public class Multiplier {
    private double m1 = 0.196951;
    private double mBlock = 0.069727;
    private double m2 = 0.200957;
    private double m3 = 0.133725;
    private double m4 = 0.043328 + 0.009764;
    private double mLast = 0.22;

    // in
    private double pivotPoint = 2.65;

    private double ticksPerInch = 29;

    private double l1 = 11.3;
    private double l2 = 11.3;
    private double l3 = 10;
    private double l4 = 10;
    private double lLast = 1;

    public double calculateMultiplier(int extendLength) {
        double lengthInches = extendLength / ticksPerInch;
        double total = m1 * (l1/2) +
                mBlock * (l1-1) +
                m2 * (Math.min(lengthInches + l1/2, l1+l2/2) + 0.3) +
                m3 * (Math.min(lengthInches + l1/2, l1+l2+l3/2) +0.6 )+
                m4 * (Math.min(lengthInches + l1/2, l1+l2+l3+l4/2) + 0.8) +
                mLast * (Math.min(lengthInches + l1/2, l1+l2+l3+l4+lLast/2) + 0.9);
        return (total / (m1+m2+m3+m4+mLast+mBlock)) - pivotPoint;
    }
}
