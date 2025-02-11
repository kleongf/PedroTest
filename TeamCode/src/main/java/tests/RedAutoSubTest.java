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
import shared.Claw;
import shared.Extend;
import shared.Intake;
import shared.Lift;
import static shared.Constants.*;


@Autonomous(name = "Autonomous Red Spec claw")
public class RedAutoSubTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    /** Start Pose of our robot */
    private final Pose startPose = new Pose(135, 72, Math.toRadians(0));

    /** Scoring Poses of our robot. It is facing the submersible. */
    private final Pose scorePose = new Pose(105, 72, Math.toRadians(0));

    private final Pose scoreOnePose = new Pose(105, 70, Math.toRadians(0));

    private final Pose scoreTwoPose = new Pose(105, 68, Math.toRadians(0));

    private final Pose scoreThreePose = new Pose(105, 66, Math.toRadians(0));

    // TODO: MAKE THESE YOURSELF
    /** Pick up blocks */
    private final Pose pickup1Pose = new Pose(112, 108, Math.toRadians(130));

    private final Pose pickup2Pose = new Pose(112, 118, Math.toRadians(130));

    private final Pose pickup3Pose = new Pose(112, 128, Math.toRadians(130));

    // once blocks are picked up they need to be placed in the taped off area

    private final Pose placePickup1Pose = new Pose(112, 113, Math.toRadians(50));

    private final Pose placePickup2Pose = new Pose(112, 123, Math.toRadians(50));

    private final Pose placePickup3Pose = new Pose(120, 120, Math.toRadians(50));

    private final Pose pickUpZonePose = new Pose(135, 114, Math.toRadians(0));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, placePickup1, placePickup2, placePickup3, moveBack1, moveBack2, moveBack3, scorePickup1, scorePickup2, scorePickup3, goToPickupZone;

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
    Claw intake;

    public void prepareScore() {
        if (actionTimer.getElapsedTimeSeconds() < 0.2) {
            intake.score();
        } else if (actionTimer.getElapsedTimeSeconds() < 0.4) {
            lift.setTarget(105);
        }
    }

    public void score() {
        if (actionTimer.getElapsedTimeSeconds() < 0.2) {
            extend.setTarget(200);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.4) {
            intake.open();
        }
    }


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(132, 96, Point.CARTESIAN),
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        placePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(placePickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), placePickup1Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(placePickup1Pose),
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(placePickup1Pose.getHeading(), pickup2Pose.getHeading())
                .build();

        placePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(placePickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), placePickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(placePickup2Pose),
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(placePickup2Pose.getHeading(), pickup3Pose.getHeading())
                .build();

        placePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(pickup3Pose),
                                new Point(115, 120, Point.CARTESIAN),
                                new Point(placePickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), placePickup3Pose.getHeading())
                .build();


        goToPickupZone = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(placePickup3Pose),
                                new Point(pickUpZonePose)
                        )
                )
                .setLinearHeadingInterpolation(placePickup3Pose.getHeading(), pickUpZonePose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickUpZonePose),
                                new Point(scoreOnePose)
                        )
                )
                .setLinearHeadingInterpolation(pickUpZonePose.getHeading(), scoreOnePose.getHeading())
                .build();

        moveBack1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scoreOnePose),
                                new Point(pickUpZonePose)
                        )
                )
                .setLinearHeadingInterpolation(scoreOnePose.getHeading(), pickUpZonePose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickUpZonePose),
                                new Point(scoreTwoPose)
                        )
                )
                .setLinearHeadingInterpolation(pickUpZonePose.getHeading(), scoreTwoPose.getHeading())
                .build();

        moveBack2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scoreTwoPose),
                                new Point(pickUpZonePose)
                        )
                )
                .setLinearHeadingInterpolation(scoreTwoPose.getHeading(), pickUpZonePose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickUpZonePose),
                                new Point(scoreThreePose)
                        )
                )
                .setLinearHeadingInterpolation(pickUpZonePose.getHeading(), scoreThreePose.getHeading())
                .build();

        moveBack3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scoreThreePose),
                                new Point(pickUpZonePose)
                        )
                )
                .setLinearHeadingInterpolation(scoreThreePose.getHeading(), pickUpZonePose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                actionTimer.resetTimer();
                setPathState(1);
                break;
            case 1:
                if (follower.isBusy()) {
                    prepareScore();
                }

                if(!follower.isBusy()) {
                    /* Score Preload */
                    score();
                    if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                        // lift down and extend
                        follower.followPath(grabPickup1,false);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                        lift.setTarget(10);
                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                        extend.setTarget(250);
                    }
                }
                if(!follower.isBusy()) {
                    intake.close();
                    follower.followPath(placePickup1,false);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    intake.open();
                    follower.followPath(grabPickup2, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake.close();
                    follower.followPath(placePickup2,false);
                    setPathState(5);
                }
                break;
            case 5:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                        intake.open();
                        follower.followPath(grabPickup3, false);
                        setPathState(6);

                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    intake.close();
                    follower.followPath(placePickup3,false);
                    setPathState(7);
                }
                break;
            case 7:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    intake.open();
                    follower.followPath(goToPickupZone, false);
                    setPathState(8);

                }
                break;
            case 8:
                if (follower.isBusy()) {
                    extend.setTarget(50);
                    lift.setTarget(40);
                    intake.dropOff();
                }
                if (!follower.isBusy()) {
                    intake.close();
                    follower.followPath(scorePickup1,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    score();
                    if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                        follower.followPath(moveBack1, true);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (follower.isBusy()) {
                    extend.setTarget(50);
                    lift.setTarget(40);
                    intake.dropOff();
                }
                if(!follower.isBusy()) {
                    intake.close();
                    follower.followPath(scorePickup2,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    score();
                    if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                        follower.followPath(moveBack2, true);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if (follower.isBusy()) {
                    extend.setTarget(50);
                    lift.setTarget(40);
                    intake.dropOff();
                }
                if(!follower.isBusy()) {
                    intake.close();
                    follower.followPath(scorePickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.isBusy()) {
                    actionTimer.resetTimer();
                }
                if(!follower.isBusy()) {
                    score();
                    if (actionTimer.getElapsedTimeSeconds() > 0.6) {
                        follower.followPath(moveBack3, true);
                        setPathState(14);
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
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

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Claw(hardwareMap);

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

