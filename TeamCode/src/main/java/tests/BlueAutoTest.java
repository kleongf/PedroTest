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

// other idea: we can move arm and pivot while driving?
@Autonomous(name = "Autonomous Blue Testing")
public class BlueAutoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 108, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(19.5, 124.5, Math.toRadians(-45));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(42, 112, Math.toRadians(80));

    private final Pose pickup2Pose = new Pose(42, 122, Math.toRadians(80));

    private final Pose pickup3Pose = new Pose(42, 132, Math.toRadians(80));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    // it seems like the first and last paths should be paths, not chains.
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

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
            lift.setTarget(180);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
            extend.setTarget(700);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
            intake.IntakeUp();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
            intake.IntakeReverse();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.5) {
            intake.IntakeDown();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.8) {
            extend.setTarget(40);
        } else if (actionTimer.getElapsedTimeSeconds() < 2.1) {
            lift.setTarget(86);
        }
    }
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(31.000, 105.000, Point.CARTESIAN),
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup1Pose),
                                new Point(31.000, 105.000, Point.CARTESIAN),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(31.000, 115.000, Point.CARTESIAN),
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup2Pose),
                                new Point(31.000, 115.000, Point.CARTESIAN),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(31.000, 125.000, Point.CARTESIAN),
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                new Point(31.000, 125.000, Point.CARTESIAN),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();


        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"

                TODO: maybe here we reset
                */
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }


                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    score();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if (actionTimer.getElapsedTimeSeconds() > 2.4) {
                        intake.IntakeForward();
                        follower.followPath(grabPickup1,false);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */

                if(!follower.isBusy()) {
                    /* Grab Sample */
                    // it should automatically grab it so i guess i shouldnt worry?
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
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
                        intake.IntakeForward();
                        follower.followPath(grabPickup2, false);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
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
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
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
                        setPathState(8);
                    }
                }
                break;
            case 8:
            /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* TODO: PARK IN WHITE LINE */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
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

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}


