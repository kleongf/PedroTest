package opmodes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import robot.Alliance;
import robot.OpModeType;
import robot.Robot;
import shared.Incrementor;

@Autonomous(name = "Red Sample: 0+4")
public class SampleRed extends OpMode {
    public Robot robot;
    private Follower follower;
    private Incrementor incrementor;

    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(135, 32, Math.toRadians(180));
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(116, 23, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(116, 13, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(118, 13, Math.toRadians(208));

    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2= follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

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
                follower.followPath(scorePreload, true);
                robot.startPrepare();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && !robot.isPrepareBusy()) {
                    robot.startDeposit();
                    setPathState(2);
                }
                break;
            case 2:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabPickup1, true);
                    robot.startRetract();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && !robot.isRetractBusy()) {
                    robot.startGrab();
                    setPathState(4);
                }
                break;
            case 4:
                if (!robot.isGrabBusy()) {
                    follower.followPath(scorePickup1, true);
                    robot.startPrepare();
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && !robot.isPrepareBusy()) {
                    robot.startDeposit();
                    setPathState(6);
                }
                break;
            case 6:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabPickup2, true);
                    robot.startRetract();
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && !robot.isRetractBusy()) {
                    robot.startGrab();
                    setPathState(8);
                }
                break;
            case 8:
                if (!robot.isGrabBusy()) {
                    follower.followPath(scorePickup2, true);
                    robot.startPrepare();
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy() && !robot.isPrepareBusy()) {
                    robot.startDeposit();
                    setPathState(10);
                }
                break;
            case 10:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabPickup3, true);
                    robot.startRetract();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && !robot.isRetractBusy()) {
                    robot.startGrab();
                    setPathState(12);
                }
                break;
            case 12:
                if (!robot.isGrabBusy()) {
                    follower.followPath(scorePickup3, true);
                    robot.startPrepare();
                    setPathState(13);
                }
                break;
        }
    }

    /** These change the states of the paths and actions and reset the timers of switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        robot.loop();
        autonomousPathUpdate();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1 , gamepad2, telemetry, Alliance.RED, OpModeType.AUTONOMOUS, startPose);
        follower = robot.getFollower();
        incrementor = robot.getIncrementor();
        pathTimer = new Timer();
        robot.setIsPickup(true);
        // TODO: Experiment with different manual power speeds to make it faster
        // extend.setManualPower(0.7);

    }

    /** Driver selects four positions to go to. **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}