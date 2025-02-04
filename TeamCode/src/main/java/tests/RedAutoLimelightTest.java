package tests;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import shared.Extend;
import shared.Intake;
import shared.Lift;
import vision.Detector;

import static shared.Constants.*;

// other idea: we can move arm and pivot while driving?
@Autonomous(name = "Autonomous Red Limelight Testing")
public class RedAutoLimelightTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    private AnalogInput analogEncoder;

    /** Start Pose of our robot */
    final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    // figure out how much to go from here

    final Pose submersibleStartPose = new Pose(72, 42, Math.toRadians(90));

    PathChain goToSubmersible, goToPickup, grabPickup, scorePickup;

    Lift lift;
    Extend extend;
    Intake intake;
    Detector detector;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        goToSubmersible = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(80, 24, Point.CARTESIAN),
                                new Point(submersibleStartPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersibleStartPose.getHeading())
                .build();
    }

    public void buildDynamicPaths(double x, double y) {
        final Pose submersibleEndPose = new Pose(submersibleStartPose.getX() + x, submersibleStartPose.getY() + y, Math.toRadians(90));

        goToPickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(submersibleStartPose),
                                new Point(submersibleEndPose)
                        )
                )
                .setLinearHeadingInterpolation(submersibleStartPose.getHeading(), submersibleEndPose.getHeading())
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(submersibleEndPose),
                                new Point(new Pose(submersibleEndPose.getX(), submersibleEndPose.getY()+4, submersibleEndPose.getHeading()))
                        )
                )
                .setLinearHeadingInterpolation(submersibleEndPose.getHeading(), submersibleEndPose.getHeading())
                .build();


        scorePickup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersibleEndPose),
                                new Point(80, 24, Point.CARTESIAN),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersibleEndPose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(goToSubmersible);
                setPathState(1);
                break;
            case 1:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }

                if(!follower.isBusy()) {
                    if (detector.getPositionX() != 0 && detector.getPositionY() != 0) {
                        buildDynamicPaths(detector.getPositionX(), detector.getPositionY());
                        follower.followPath(goToPickup,false);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                        lift.setTarget(ANGLE_MID);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                        extend.setTarget(EXTEND_MID);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
                        intake.IntakeForward();
                    } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
                        intake.IntakeDown();
                    } else if (actionTimer.getElapsedTimeSeconds() < 1.5) {
                        follower.followPath(grabPickup, false);
                        setPathState(3);
                    }
                }
            case 3:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }

                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                        intake.IntakeUp();
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                        lift.setTarget(ANGLE_ZERO);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
                        extend.setTarget(EXTEND_ZERO);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.2) {
                        follower.followPath(scorePickup, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        intake.loop();
        lift.loop();
        extend.loop();
        detector.loop();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        detector = new Detector();
        opmodeTimer.resetTimer();

        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

