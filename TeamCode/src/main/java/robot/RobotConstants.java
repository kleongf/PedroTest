package robot;
public final class RobotConstants {
    private RobotConstants() {}

    public static final int LIFT_DEFAULT = 59;
    public static final int LIFT_SUBMERSIBLE = 64;
    public static final int LIFT_UP = 158;
    public static final int EXTEND_MAX = 700;
    public static final int EXTEND_DEFAULT = 200;
    public static final int EXTEND_ZERO = 0;

    // HARD STOPS
    public static final int EXTEND_STOP_MAX = 760; // made larger for climb
    public static final int EXTEND_STOP_MIN = 0;
    public static final int ANGLE_STOP_MAX = 170;
    public static final int ANGLE_STOP_MIN = 55;
}