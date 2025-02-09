package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.pathgen.BezierPoint;
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
import vision.YellowPipeline;

import static shared.Constants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous Red Claw CV Testing")
public class RedSampleClawCV extends OpMode {
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

    private final Pose pickup3Pose = new Pose(123.5, 14, Math.toRadians(216));

    private final Pose submersibleStartPose = new Pose(72, 30, Math.toRadians(90));

    private final Pose submersibleEndPose = new Pose(72, 52, Math.toRadians(90));

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, trimY;
    private PathChain goToSubmersible, grabPickup, scorePickup;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    public AnalogInput analogEncoder;

    Lift lift;
    Extend extend;
    Claw intake;

    OpenCvCamera phoneCam;
    YellowPipeline yellowPipeline;

    public void score() {
        if (actionTimer.getElapsedTimeSeconds() < 0.3) {
            lift.setTarget(ANGLE_UP_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
            extend.setTarget(EXTEND_HIGH_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
            intake.score();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
            intake.open();
        } else if (actionTimer.getElapsedTimeSeconds() < 1.5) {
            extend.setTarget(EXTEND_DEFAULT_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 1.8) {
            lift.setTarget(ANGLE_ZERO);
            intake.submersibleDown();
        }
    }

    public void grabBlock() {
        if (actionTimer.getElapsedTimeSeconds() < 0.2) {
            extend.setTarget(600);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.5) {
            lift.setTarget(9.5);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.8) {
            intake.close();
        } else if (actionTimer.getElapsedTimeSeconds() < 1) {
            lift.setTarget(ANGLE_ZERO);
        } else if (actionTimer.getElapsedTimeSeconds() < 1.3) {
            extend.setTarget(0);
        }
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        goToSubmersible = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(75, 23, Point.CARTESIAN),
                                new Point(submersibleStartPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersibleStartPose.getHeading())
                .build();

        trimY = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(submersibleStartPose),
                                new Point(submersibleEndPose)
                        )
                )
                .setLinearHeadingInterpolation(submersibleStartPose.getHeading(), submersibleEndPose.getHeading())
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
                        follower.turnToDegrees(165);
                        actionTimer.resetTimer();
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (follower.isBusy()) {actionTimer.resetTimer();}
                if (!follower.isBusy()) {
                    grabBlock();
                    if (actionTimer.getElapsedTimeSeconds() < 1.5) {
                        follower.turnToDegrees(135);
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
                        follower.turnToDegrees(192);
                        actionTimer.resetTimer();
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (follower.isBusy()) {actionTimer.resetTimer();}
                if (!follower.isBusy()) {
                    grabBlock();
                    if (actionTimer.getElapsedTimeSeconds() < 1.5) {
                        follower.turnToDegrees(135);
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
                        follower.turnToDegrees(216);
                        actionTimer.resetTimer();
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (follower.isBusy()) {actionTimer.resetTimer();}
                if (!follower.isBusy()) {
                    grabBlock();
                    if (actionTimer.getElapsedTimeSeconds() < 1.5) {
                        follower.turnToDegrees(135);
                        actionTimer.resetTimer();
                        setPathState(7);
                    }
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
                lift.setTarget(ANGLE_ZERO);
                follower.followPath(goToSubmersible, false);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    extend.setTarget(500);
                    // slowing it down but maybe it can go faster
                    follower.setMaxPower(0.36);
                    follower.followPath(trimY, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.isBusy()) {
                    if (yellowPipeline.isBlockDetected()) {
                        scorePickup = follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Point(follower.getPose()),
                                                new Point(75, 23, Point.CARTESIAN),
                                                new Point(scorePose)
                                        )
                                )
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                                .build();
                        follower.breakFollowing();
                        actionTimer.resetTimer();
                        setPathState(11);
                    }
                } else {
                    scorePickup = follower.pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            new Point(follower.getPose()),
                                            new Point(75, 23, Point.CARTESIAN),
                                            new Point(scorePose)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    follower.breakFollowing();
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                    if (yellowPipeline.getOrientation() == 1) {intake.spinHorizontal();}
                    lift.setTarget(9);
                } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
                    intake.close();
                } else if (actionTimer.getElapsedTimeSeconds() < 1) {
                    lift.setTarget(ANGLE_ZERO);
                } else if (actionTimer.getElapsedTimeSeconds() < 1.3) {
                    extend.setTarget(0);
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
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
        follower.update();
        autonomousPathUpdate();

        lift.loop();
        extend.loop();

        // Feedback to Driver Hub
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Detected", yellowPipeline.isBlockDetected());
        telemetry.addData("Orientation", yellowPipeline.getOrientation());
        telemetry.addData("Timer", actionTimer.getElapsedTimeSeconds());
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

        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        yellowPipeline = new YellowPipeline();
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.setPipeline(yellowPipeline);
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error");
            }
        });
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

