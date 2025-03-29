package opmodes.autonomous;

import static com.pedropathing.util.Constants.setConstants;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import robot.SpecRobot;

@Autonomous(name = "Pedro 5+0")
public class SpecAuto extends OpMode {
    public SpecRobot robot;
    private Follower follower;

    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(8, 72, Math.toRadians(180));
    private final Pose grabPose = new Pose(7, 30, Math.toRadians(180));
    private final Pose scorePose = new Pose(39, 72, Math.toRadians(180));

    private final Pose score1Pose = new Pose(39, 70, Math.toRadians(180));
    private final Pose score2Pose = new Pose(39, 68, Math.toRadians(180));
    private final Pose score3Pose = new Pose(39, 66, Math.toRadians(180));
    private final Pose score4Pose = new Pose(39, 64, Math.toRadians(180));

    private PathChain scorePreload, push, grabSpec2, scoreSpec1, scoreSpec2, grabSpec3, scoreSpec3, grabSpec4, scoreSpec4;
    public void buildPaths() {
        // TODO: set custom zpams
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        push = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(40.000, 72.000, Point.CARTESIAN),
                                new Point(6, 29.156, Point.CARTESIAN),
                                new Point(65.778, 35.022, Point.CARTESIAN),
                                new Point(54.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(20)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(54.000, 25.000, Point.CARTESIAN),
                                new Point(28.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(20)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(28.000, 25.000, Point.CARTESIAN),
                                new Point(64.711, 25.422, Point.CARTESIAN),
                                new Point(54.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(20)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(54.000, 15.000, Point.CARTESIAN),
                                new Point(28.000, 15.000, Point.CARTESIAN)
                        )
                )
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(20)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(28.000, 15.000, Point.CARTESIAN),
                                new Point(64.000, 16.356, Point.CARTESIAN),
                                new Point(54.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(20)
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(54.000, 8.000, Point.CARTESIAN),
                                new Point(8.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndVelocityConstraint(10)
                .build();

        scoreSpec1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(8, 8, Point.CARTESIAN),
                                new Point(score1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .build();

        grabSpec2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score2Pose),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        scoreSpec2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score2Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();


        grabSpec3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score2Pose),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        scoreSpec3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score3Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        grabSpec4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(score3Pose),
                                new Point(grabPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        scoreSpec4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(grabPose),
                                new Point(score4Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                robot.startPrepareScore();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    robot.startDeposit();
                    setPathState(2);
                }
                break;
            case 2:
                if (!robot.isDepositBusy()) {
                    robot.startPrepareGrab();
                    follower.followPath(push, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    robot.startGrab();
                    setPathState(15);
                }
                break;
            case 15:
                if (!robot.isGrabBusy()) {
                    robot.startPrepareScore();
                    follower.followPath(scoreSpec1, true);
                    setPathState(16);
                }
                break;
            // JUMP
            case 16:
                if (!follower.isBusy() && !robot.isPrepareScoreBusy()) {
                    robot.startDeposit();
                    setPathState(21);
                }
                break;
            case 21:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabSpec2);
                    robot.startPrepareGrab();
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy() && !robot.isPrepareGrabBusy()) {
                    robot.startGrab();
                    setPathState(23);
                }
                break;
            case 23:
                if (!robot.isGrabBusy()) {
                    robot.startPrepareScore();
                    follower.followPath(scoreSpec2, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy() && !robot.isPrepareScoreBusy()) {
                    robot.startDeposit();
                    setPathState(25);
                }
                break;
            case 25:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabSpec3);
                    robot.startPrepareGrab();
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy() && !robot.isPrepareGrabBusy()) {
                    robot.startGrab();
                    setPathState(27);
                }
                break;
            case 27:
                if (!robot.isGrabBusy()) {
                    robot.startPrepareScore();
                    follower.followPath(scoreSpec3, true);
                    setPathState(28);
                }
                break;
            case 28:
                if (!follower.isBusy() && !robot.isPrepareScoreBusy()) {
                    robot.startDeposit();
                    setPathState(29);
                }
                break;

            case 29:
                if (!robot.isDepositBusy()) {
                    follower.followPath(grabSpec4);
                    robot.startPrepareGrab();
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy() && !robot.isPrepareGrabBusy()) {
                    robot.startGrab();
                    setPathState(31);
                }
                break;
            case 31:
                if (!robot.isGrabBusy()) {
                    robot.startPrepareScore();
                    follower.followPath(scoreSpec4, true);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy() && !robot.isPrepareScoreBusy()) {
                    robot.startDeposit();
                    setPathState(33);
                }
                break;

            case 33:
                if (!robot.isDepositBusy()) {
                    setPathState(-1);
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
        follower.update();
        autonomousPathUpdate();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot = new SpecRobot(hardwareMap, telemetry);

        setConstants(LConstants.class, FConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        pathTimer = new Timer();
    }

    @Override
    public void init_loop() {}

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

