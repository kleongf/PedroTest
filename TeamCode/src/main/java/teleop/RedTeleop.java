package teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import shared.Constants;
import shared.Extend;
import shared.Intake;
import shared.Lift;
import static shared.Constants.* ;

@TeleOp(name = "Red TeleOp")
public class RedTeleop extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_UP_HIGH,
        EXTEND_RETRACT,
        LIFT_DOWN,
        WAIT,
    }

    public enum IntakeState {
        INTAKE_START,
        INTAKE_UP,
        INTAKE_LIFT_DOWN
    }

    public enum DriveState {
        DRIVE_START,
        DRIVE_WAIT
    }

    LiftState liftState = LiftState.LIFT_START;
    IntakeState intakeState = IntakeState.INTAKE_START;
    DriveState driveState = DriveState.DRIVE_START;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    private AnalogInput analogEncoder;

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    Lift lift;
    Extend extend;
    Intake intake;


    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    private boolean leftStickPressed = false;
    private boolean rightStickPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftBumperPressed = false;
    private boolean crossPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean rightBumperPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean trianglePressed = false;

    private Follower follower;
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);
        com.pedropathing.util.Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(scorePose);

        actionTimer.reset();
    }
    /*
    Lt mid distance (done)
    LB max distance out (done)
    LSB intake running and intake flip down (done)
    RSB eject (done)

    D-pad up get ready for climb (need to test climb first)
    D-pad down Climb (need to test climb first)
    A button retract and flip up intake (done, also works for retracting when grabbing sample)

    Right bumper: HANG
     */

    public void loop() {
        switch (liftState) {
            case LIFT_START:
                if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
                    lift.setTarget(ANGLE_UP);
                    actionTimer.reset();
                    liftState = LiftState.LIFT_UP_HIGH;
                } else if (gamepad1.cross && !crossPressed) {
                    if (lift.getTarget() > ANGLE_ZERO + 10) {
                        extend.setTarget(EXTEND_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.EXTEND_RETRACT;
                    } else {
                        lift.setTarget(ANGLE_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.LIFT_DOWN;
                    }
                } else if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
                    extend.setTarget(EXTEND_MID);
                } else if (gamepad1.left_bumper && !leftBumperPressed) {
                    extend.setTarget(EXTEND_MAX);
                } else if (gamepad1.right_bumper && !rightBumperPressed) {
                    lift.setTarget(ANGLE_HANG);
                    // we need to increase extension max for trimming
                }
                break;

            case LIFT_UP_HIGH:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_HIGH);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            case WAIT:
                if (actionTimer.seconds() >= 0.3) {
                    actionTimer.reset();
                    liftState = LiftState.LIFT_START;
                }
                break;

            case LIFT_DOWN:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_ZERO);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            case EXTEND_RETRACT:
                if (actionTimer.seconds() >= 0.3) {
                    lift.setTarget(ANGLE_ZERO);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            default:
                liftState = LiftState.LIFT_START;
        }

        switch (intakeState) {
            case INTAKE_START:
                if (gamepad1.left_stick_button && !leftStickPressed) {
                    intake.IntakeForward();
                    intake.IntakeDown();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_LIFT_DOWN;
                } else if (gamepad1.right_stick_button && !rightStickPressed) {
                    intake.IntakeReverse();
                    // then wait, then intake stop
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            case INTAKE_LIFT_DOWN:
                if (intakeTimer.seconds() >= 0.3) {
                    if (extend.getTarget() <= EXTEND_MID && extend.getTarget() > 0) {
                        lift.setTarget(ANGLE_MID);
                    } else if (extend.getTarget() <= EXTEND_MAX && extend.getTarget() > EXTEND_MID) {
                        lift.setTarget(ANGLE_MAX);
                    } else {
                        lift.setTarget(ANGLE_STOP_MIN);
                    }
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_UP;
                }
                break;

            case INTAKE_UP:
                if (!gamepad1.left_stick_button) {
                    intake.IntakeUp();
                    // stop intake when its not being used
                    intake.IntakeStop();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        switch (driveState) {
            case DRIVE_START:
                if (gamepad2.triangle && !trianglePressed) {
                    Pose currentPose = follower.getPose();
                    PathChain goToBucket = follower.pathBuilder()
                            .addPath(
                                    // maybe it should be a curve...
                                    // we don't want the bot to hit stuff
                                    new BezierCurve(
                                            new Point(currentPose),
                                            new Point(80, 22),
                                            new Point(scorePose)
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                            .build();
                    follower.followPath(goToBucket);
                    driveState = DriveState.DRIVE_WAIT;
                }
                break;

            case DRIVE_WAIT:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    driveState = DriveState.DRIVE_START;
                }
                break;

            default:
                driveState = DriveState.DRIVE_START;
        }

        // dpad trimming
        if (gamepad2.dpad_up && !dpadUpPressed) {
            extend.setTarget(extend.getTarget() + TRIM_AMOUNT);
        } else if (gamepad2.dpad_down && !dpadDownPressed) {
            extend.setTarget(extend.getTarget() - TRIM_AMOUNT);
        }

        extend.loop();
        lift.loop();
        intake.loop();

        telemetry.update();
        follower.update();

        rightTriggerPressed = gamepad1.right_trigger > 0.5;
        leftBumperPressed = gamepad1.left_bumper;
        leftStickPressed = gamepad1.left_stick_button;
        rightStickPressed = gamepad1.right_stick_button;
        leftTriggerPressed = gamepad1.left_trigger > 0.5;
        crossPressed = gamepad1.cross;
        dpadDownPressed = gamepad1.dpad_down;
        dpadUpPressed = gamepad1.dpad_up;
        rightBumperPressed = gamepad1.right_bumper;
        trianglePressed = gamepad2.triangle;

        // Drive logic (unchanged)
        // TODO: actually i just switched the controls lets see if it wroks
        double y = -gamepad2.left_stick_x;
        double x = gamepad2.left_stick_y * 1.1;
        double rx = gamepad2.right_stick_y;

        if (Math.abs(y) > 0.2 || Math.abs(x) > 0.2 || Math.abs(rx) > 0.2) {
            follower.breakFollowing();
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // slow mo
        boolean lowPower = gamepad2.left_trigger > 0.5;

        // i think this will solve the conflicting issues
        if (!follower.isBusy()) {
            frontLeft.setPower(lowPower ? 0.5 * frontLeftPower : frontLeftPower);
            backLeft.setPower(lowPower ? 0.5 * backLeftPower : backLeftPower);
            frontRight.setPower(lowPower ? 0.5 * -frontRightPower : -frontRightPower);
            backRight.setPower(lowPower ? 0.5 * -backRightPower : -backRightPower);
        }
    }
}


