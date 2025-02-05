package shared;

public final class Constants {
    private Constants() {}

    // maybe we put poses in here?

    // AUTONOMOUS
    // TODO: Change these angles to be updated to current offset of 10
    public static final double ANGLE_DOWN_AUTO = 5;
    public static final int ANGLE_UP_AUTO = 110;
    public static final int EXTEND_HIGH_AUTO = 710;
    public static final int EXTEND_DEFAULT_AUTO = 40;

    // TELEOP
    public static final int ANGLE_UP = 100;
    public static final int ANGLE_ZERO = 11;
    public static final int ANGLE_HANG = 110;
    public static final double ANGLE_MID = 8;
    // problem is that extension might be a bit too long, angle stops working
    // find another extension that might work
    public static final double ANGLE_MAX = 10;
    public static final int EXTEND_HIGH = 700;
    public static final int EXTEND_ZERO = 0;
    public static final int EXTEND_MID = 350;
    public static final int EXTEND_MAX = 700;
    public static final int TRIM_AMOUNT = 30;

    // HARD STOPS
    public static final int EXTEND_STOP_MAX = 760; // made larger for climb
    public static final int EXTEND_STOP_MIN = 0;
    public static final int ANGLE_STOP_MAX = 120;
    public static final int ANGLE_STOP_MIN = 3;

    // INTAKE
    public static final double SERVO_UP = 0.8;
    public static final double SERVO_DOWN = 0.22;
    public static final double SERVO_FORWARD = -0.3;
    public static final double SERVO_REVERSE = 0.3;
}
