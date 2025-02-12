package teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import shared.Claw;
import shared.Extend;
import shared.Intake;
import shared.Lift;
import static shared.Constants.* ;

@TeleOp(name = "Red Specimen TeleOp")
public class RedSpecimenTeleop extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_UP_HIGH,
        SCORE,
        EXTEND_RETRACT,
        LIFT_DOWN,
        WAIT,
    }

    public enum IntakeState {
        INTAKE_START,
        INTAKE_UP,
        INTAKE_LIFT_DOWN,
        INTAKE_OPEN
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
    private AnalogInput analogEncoder;

    Lift lift;
    Extend extend;
    Claw intake;


    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    private Gamepad currentGamepad1 = new Gamepad(), currentGamepad2 = new Gamepad(), previousGamepad1 = new Gamepad(), previousGamepad2 = new Gamepad();

    private Follower follower;
    private final Pose scorePose = new Pose(106, 72, Math.toRadians(0));
    private final Pose dropOffPose = new Pose(135, 108, Math.toRadians(0));

    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");


        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Claw(hardwareMap);
        com.pedropathing.util.Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(scorePose);
        follower.startTeleopDrive();
        actionTimer.reset();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        switch (liftState) {
            case LIFT_START:
                if (gamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger < 0.5) {
                    intake.score();
                    lift.setTarget(ANGLE_UP);
                    actionTimer.reset();
                    liftState = LiftState.LIFT_UP_HIGH;
                } else if (gamepad1.cross && !previousGamepad1.cross) {
                    if (lift.getTarget() > ANGLE_ZERO + 10) {
                        extend.setTarget(EXTEND_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.EXTEND_RETRACT;
                    } else {
                        lift.setTarget(ANGLE_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.LIFT_DOWN;
                    }
                } else if (gamepad1.left_trigger > 0.5 && previousGamepad1.left_trigger < 0.5) {
                    extend.setTarget(EXTEND_MID);
                } else if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    extend.setTarget(EXTEND_MAX);
                } else if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                    // this is for the drop off zone
                    lift.setTarget(45);
                    extend.setTarget(40);
                    intake.dropOff();
                }
                break;

            case LIFT_UP_HIGH:
                if (actionTimer.seconds() >= 0.3) {
                    // changed to score
                    extend.setTarget(200);
                    actionTimer.reset();
                    liftState = LiftState.SCORE;
                }
                break;

            case SCORE:
                if (actionTimer.seconds() > 0.2) {
                    intake.open();
                    liftState = LiftState.WAIT;
                }

            case WAIT:
                if (actionTimer.seconds() >= 0.2) {
                    actionTimer.reset();
                    liftState = LiftState.LIFT_START;
                }
                break;

            case LIFT_DOWN:
                if (actionTimer.seconds() >= 0.2) {
                    extend.setTarget(EXTEND_ZERO);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            case EXTEND_RETRACT:
                if (actionTimer.seconds() >= 0.2) {
                    lift.setTarget(ANGLE_ZERO);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            default:
                liftState = LiftState.LIFT_START;
        }

        switch (intakeState) {
            // left stick: open, down, close on release
            // right stick: open, close on release
            case INTAKE_START:
                if (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
                    intake.open();
                    intake.submersibleDown();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_LIFT_DOWN;
                } else if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
                    intake.open();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_OPEN;
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
                    if (!gamepad1.left_stick_button) {
                        intake.close();
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_UP;
                    }
                }
                break;

            case INTAKE_UP:
                if (intakeTimer.seconds() > 0.1) {
                    intake.submersibleUp();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;
            case INTAKE_OPEN:
                if (!gamepad1.right_stick_button) {
                    intake.close();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        switch (driveState) {
            case DRIVE_START:
                if (gamepad2.triangle && !previousGamepad2.triangle) {
                    Pose currentPose = follower.getPose();
                    PathChain goToSubmersible = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose),
                                            new Point(scorePose)
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                            .build();
                    follower.followPath(goToSubmersible);
                    driveState = DriveState.DRIVE_WAIT;
                } else if (gamepad2.square && !previousGamepad2.square) {
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
        if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            extend.setTarget(extend.getTarget() + TRIM_AMOUNT);
        } else if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
            extend.setTarget(extend.getTarget() - TRIM_AMOUNT);
        }

        // toggle direction: move left: horizontal, move up: vertical
        if (Math.abs(gamepad1.left_stick_y) > 0.8) {
            intake.spinHorizontal();
        } else if (Math.abs(gamepad1.left_stick_x) > 0.8) {
            intake.spinVertical();
        }

        double speed = gamepad2.left_trigger > 0.5 ? 0.5 : 1;

        extend.loop();
        lift.loop();

        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad2.left_stick_y * speed, -gamepad2.left_stick_x * speed, -gamepad2.right_stick_x * speed, true);
        }
        telemetry.update();
        follower.update();

    }
}
