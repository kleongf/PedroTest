package shared;

public final class Constants {
    private Constants() {}

    // maybe we put poses in here?

    // AUTONOMOUS
    public static final int ANGLE_DOWN_AUTO = 86;
    public static final int ANGLE_UP_AUTO = 180;
    public static final int EXTEND_HIGH_AUTO = 700;
    public static final int EXTEND_DEFAULT_AUTO = 40;

    // TELEOP
    public static final int ANGLE_UP = 180;
    public static final int ANGLE_ZERO = 90;
    public static final int ANGLE_HANG = 190;
    public static final int ANGLE_MID = 88;
    public static final int ANGLE_MAX = 89;
    public static final int EXTEND_HIGH = 700;
    public static final int EXTEND_ZERO = 0;
    public static final int EXTEND_MID = 350;
    public static final int EXTEND_MAX = 700;
    public static final int TRIM_AMOUNT = 30;

    // HARD STOPS
    public static final int EXTEND_STOP_MAX = 720;
    public static final int EXTEND_STOP_MIN = 0;
    public static final int ANGLE_STOP_MAX = 200;
    public static final int ANGLE_STOP_MIN = 83;

    // INTAKE
    public static final double SERVO_UP = 0.8;
    public static final double SERVO_DOWN = 0.22;
    public static final double SERVO_FORWARD = -0.3;
    public static final double SERVO_REVERSE = 0.3;
}
