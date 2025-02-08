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

import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Mat;

@TeleOp(name = "Red Spec TeleOp")
public class RedTeleop extends OpMode {

    public enum RobotState {
        START,
        DROPOFF,
        SCORE,
        RETRACT,
        DOWN,
    }

    public enum DriveState {
        DRIVE_START,
        DRIVE_WAIT
    }

    RobotState robotState = RobotState.START;
    DriveState driveState = DriveState.DRIVE_START;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    private AnalogInput analogEncoder;

    Lift lift;
    Extend extend;
    Intake intake;


    ElapsedTime actionTimer = new ElapsedTime();

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
    private boolean squarePressed = false;

    private Follower follower;
    private final Pose submersiblePose = new Pose(106, 72, Math.toRadians(0));
    private final Pose dropOffPose = new Pose(135, 108, Math.toRadians(0));

    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);
        com.pedropathing.util.Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(submersiblePose);
        follower.startTeleopDrive();
        actionTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        switch (robotState) {
            case START:
                // right trigger: score
                if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
                    lift.setTarget(105);
                    // should go wayyyyy down
                    intake.IntakeDown();
                    actionTimer.reset();
                    robotState = RobotState.SCORE;
                } else if (gamepad1.right_bumper && !rightBumperPressed) {
                    lift.setTarget(40);
                    extend.setTarget(40);
                    actionTimer.reset();
                    robotState = RobotState.DROPOFF;
                } else if (gamepad1.cross && !crossPressed) {
                    if (lift.getTarget() > ANGLE_ZERO + 10) {
                        extend.setTarget(EXTEND_ZERO);
                    } else {
                        lift.setTarget(ANGLE_ZERO);
                    }
                    actionTimer.reset();
                    robotState = RobotState.RETRACT;
                } else if (gamepad1.left_stick_button && !leftStickPressed) {
                    if (extend.getTarget() <= EXTEND_MID && extend.getTarget() > 0) {
                        lift.setTarget(ANGLE_MID);
                    } else if (extend.getTarget() <= EXTEND_MAX && extend.getTarget() > EXTEND_MID) {
                        lift.setTarget(ANGLE_MAX);
                    } else {
                        lift.setTarget(ANGLE_STOP_MIN);
                    }
                    intake.IntakeForward();
                    actionTimer.reset();
                    robotState = RobotState.DOWN;
                } else if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
                    extend.setTarget(EXTEND_MID);
                } else if (gamepad1.left_bumper && !leftBumperPressed) {
                    extend.setTarget(EXTEND_MAX);
                } else if (gamepad1.right_stick_button && !rightStickPressed) {
                    intake.IntakeReverse();
                }
                break;
            case RETRACT:
                if (actionTimer.seconds() > 0.3) {
                    if (extend.getTarget() == EXTEND_ZERO) {
                        lift.setTarget(ANGLE_ZERO);
                    } else {
                        extend.setTarget(EXTEND_ZERO);
                    }
                }
                break;
            case DOWN:
                if (actionTimer.seconds() > 0.3) {
                    if (!gamepad1.left_stick_button) {
                        lift.setTarget(ANGLE_ZERO);
                        intake.IntakeStop();
                        actionTimer.reset();
                        robotState = RobotState.START;
                    }
                }
                break;
            case SCORE:
                if (actionTimer.seconds() > 0.4) {
                    extend.setTarget(200);
                } else if (actionTimer.seconds() > 0.8) {
                    intake.IntakeReverse();
                } else if (actionTimer.seconds() > 1) {
                    robotState = RobotState.START;
                }
                break;
            case DROPOFF:
                if (actionTimer.seconds() > 0.4) {
                    intake.IntakeUp();
                } else if (actionTimer.seconds() > 0.8) {
                    intake.IntakeForward();
                } else if (actionTimer.seconds() > 1) {
                    robotState = RobotState.START;
                }
                break;
            default:
                robotState = RobotState.START;
        }

        switch (driveState) {
            case DRIVE_START:
                if (gamepad2.triangle && !trianglePressed) {
                    Pose currentPose = follower.getPose();
                    PathChain goToSubmersible = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose),
                                            new Point(submersiblePose)
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), submersiblePose.getHeading())
                            .build();
                    follower.followPath(goToSubmersible);
                    driveState = DriveState.DRIVE_WAIT;
                }
                if (gamepad2.square && !squarePressed) {
                    Pose currentPose = follower.getPose();
                    PathChain goToDropOff = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose),
                                            new Point(dropOffPose)
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), dropOffPose.getHeading())
                            .build();
                    follower.followPath(goToDropOff);
                    driveState = DriveState.DRIVE_WAIT;
                }
                break;

            case DRIVE_WAIT:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    follower.startTeleopDrive();
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

        // copy the gamepads you bum

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
        squarePressed = gamepad2.square;

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x, true);
        }
        telemetry.update();
        follower.update();

    }
}

