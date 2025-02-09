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
import static shared.Constants.*;

// TODO: Preset for which x-coordinates to go to (just copy from preset test)
@Autonomous(name = "Autonomous Red Claw BAD")
public class RedSampleClaw extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(135, 32, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(123.5, 18.5, Math.toRadians(165));

    private final Pose pickup2Pose = new Pose(123.5, 18.5, Math.toRadians(192));

    private final Pose pickup3Pose = new Pose(98, 14, Math.toRadians(270));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain goToSubmersible, grabPickup, scorePickup;

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

    public void score() {
        if (actionTimer.getElapsedTimeSeconds() < 0.3) {
            lift.setTarget(ANGLE_UP_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
            extend.setTarget(EXTEND_HIGH_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
            intake.IntakeUp();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
            intake.IntakeReverse();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.5) {
            intake.IntakeDown();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.8) {
            extend.setTarget(EXTEND_DEFAULT_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 2.1) {
            lift.setTarget(ANGLE_ZERO);
        }
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup3Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Score Preload */
                    score();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (actionTimer.getElapsedTimeSeconds() > 2.4) {
                        follower.holdPoint(pickup1Pose);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (follower.isBusy()) {actionTimer.resetTimer();}
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                        extend.setTarget(600);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                        intake.IntakeForward();
                    } else if (actionTimer.getElapsedTimeSeconds() < 1) {
                        extend.setTarget(0);
                    } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
                        follower.holdPoint(scorePose);
                        actionTimer.resetTimer();
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Score Sample */
                    score();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (actionTimer.getElapsedTimeSeconds() > 2.4) {
                        follower.holdPoint(pickup2Pose);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (follower.isBusy()) {actionTimer.resetTimer();}
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                        // this looks about right when you do the math
                        extend.setTarget(600);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                        intake.IntakeForward();
                    } else if (actionTimer.getElapsedTimeSeconds() < 1) {
                        extend.setTarget(0);
                    } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
                        follower.holdPoint(scorePose);
                        actionTimer.resetTimer();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Score Sample */
                    score();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (actionTimer.getElapsedTimeSeconds() > 2.4) {
                        intake.IntakeForward();
                        follower.followPath(grabPickup3, false);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    /* Score Sample */
                    score();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (actionTimer.getElapsedTimeSeconds() > 2.4) {
                        setPathState(-1);
                    }
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
        // detector.loop();

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
        opmodeTimer.resetTimer();

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

//public class GeneratedPath {
//
//    public GeneratedPath() {
//        PathBuilder builder = new PathBuilder();
//
//        builder
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(135.000, 32.000, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(123.500, 18.500, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(165))
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(123.500, 18.500, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(135))
//                .addPath(
//                        // Line 4
//                        new BezierLine(
//                                new Point(123.500, 18.500, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(192))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(123.500, 18.500, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(192), Math.toRadians(135))
//                .addPath(
//                        // Line 6
//                        new BezierLine(
//                                new Point(123.500, 18.500, Point.CARTESIAN),
//                                new Point(98.000, 14.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(98.000, 14.000, Point.CARTESIAN),
//                                new Point(123.500, 18.500, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135));
//    }
//}
//
