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

@TeleOp(name = "Red sample Teleop")
public class RedSample extends OpMode {
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
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

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
            // right trigger: lift up
            // cross: retract
            // left trigger: extend mid
            // left bumper: extend max
            case LIFT_START:
                if (gamepad1.right_trigger > 0.5 && previousGamepad1.right_trigger < 0.5) {
                    // intake.score();
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
                }
                break;

            case LIFT_UP_HIGH:
                if (actionTimer.seconds() >= 0.2) {
                    extend.setTarget(EXTEND_HIGH);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

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

        // TODO: Open, INTAKe DOWN (not the full arm), and close+flip up on release
        switch (intakeState) {
            // left stick: open, down, close on release
            // right stick: open
            case INTAKE_START:
                if (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
                    intake.open();
                    intake.submersibleDown();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_LIFT_DOWN;
                } else if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
                    intake.open();
                    intakeTimer.reset();
                }
                break;

            case INTAKE_LIFT_DOWN:
                if (intakeTimer.seconds() >= 0.2) {
                    if (extend.getCurrentPosition() > 0) {
                        lift.setTarget(10-Math.toDegrees(Math.atan(1/(12+(extend.getCurrentPosition()/29.0)))));
                    } else {
                        // extension is at 0, needs to be lifted down to the minimum angle
                        lift.setTarget(ANGLE_STOP_MIN);
                    }
                    if (!gamepad1.left_stick_button) {
                        intake.close();
                        intakeTimer.reset();
                        intakeState = IntakeState.INTAKE_UP;
                    }
//                    // it should consistently move the arm down the correct amount
//                    // 10-arctan(1/(extend.getTarget()/ticksperinch))
//                    // watch out for div 0 errors if target is 0
//                    if (extend.getTarget() <= EXTEND_MID && extend.getTarget() > 0) {
//                        lift.setTarget(ANGLE_MID);
//                    } else if (extend.getTarget() <= EXTEND_MAX && extend.getTarget() > EXTEND_MID) {
//                        lift.setTarget(ANGLE_MAX);
//                    } else {
//                        lift.setTarget(ANGLE_STOP_MIN);
//                    }
//                    if (!gamepad1.left_stick_button) {
//                        intake.close();
//                        intakeTimer.reset();
//                        intakeState = IntakeState.INTAKE_UP;
//                    }
                }
                break;

            case INTAKE_UP:
                if (intakeTimer.seconds() > 0.2) {
                    intake.score();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;
//            case INTAKE_OPEN:
//                if (!gamepad1.right_stick_button) {
//                    intake.open();
//                    intakeTimer.reset();
//                    intakeState = IntakeState.INTAKE_START;
//                }
//                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        switch (driveState) {
            // triangle: drive back
            case DRIVE_START:
                if (gamepad2.triangle && !previousGamepad2.triangle) {
                    Pose currentPose = follower.getPose();
                    PathChain goToBucket = follower.pathBuilder()
                            .addPath(
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
                    follower.startTeleopDrive();
                    driveState = DriveState.DRIVE_START;
                }
                break;

            default:
                driveState = DriveState.DRIVE_START;
        }
        // TODO: Test new trimming
        // dpad trimming: hopefully new code works? did not change for spec yet
        if (gamepad2.dpad_up) {
            extend.manual(1);
        } else if (gamepad2.dpad_down) {
            extend.manual(-1);
        }
        // on release: hold the current position
        if ((!gamepad2.dpad_up && previousGamepad2.dpad_up) || (!gamepad2.dpad_down && previousGamepad2.dpad_down)) {
            extend.setTarget(extend.getCurrentPosition());
        }

        // toggle spin: rotate the left stick anywhere violently
        if (Math.abs(gamepad1.left_stick_y) > 0.8 || Math.abs(gamepad1.left_stick_x) > 0.8)
            intake.toggleSpin();

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

