package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import shared.Claw;
import shared.Extend;
import shared.Incrementor;
import shared.Intake;
import shared.Lift;
import shared.TeleOpLift;
import vision.YellowPipeline;
import vision.YellowRedPipeline;

import static shared.Constants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Best Sample Auto")
public class SampleBest extends OpMode {
    private List<LynxModule> allHubs;
    private Follower follower;
    private Timer pathTimer, actionTimer;
    private int pathState;

    private Timer loopTimer;
    private ArrayList<Double> loopTimes;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(135, 32, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(116, 23, Math.toRadians(180));

    private final Pose pickup2Pose = new Pose(116, 13, Math.toRadians(180));

    // private final Pose pickup3Pose = new Pose(112, 16, Math.toRadians(225));

    private final Pose pickup3Pose = new Pose(118, 13, Math.toRadians(208));

    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    private PathChain submersible1, submersible2, submersible3, submersible4, scoreSubmersible1, scoreSubmersible2, scoreSubmersible3, scoreSubmersible4;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public AnalogInput analogEncoder;

    private Incrementor incrementor = new Incrementor(4);

    // TODO: test which lifts are better
    TeleOpLift lift;
    Extend extend;
    Claw intake;

    // OpenCvCamera phoneCam;
    YellowRedPipeline yellowPipeline;

    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;

    Runnable openRunnable = new Runnable() {
        @Override
        public void run() {
            intake.open();
        }
    };

    Runnable closeRunnable = new Runnable() {
        @Override
        public void run() {
            intake.close();
        }
    };

    Runnable liftUpRunnable = new Runnable() {
        @Override
        public void run() {
            lift.setTarget(ANGLE_UP_AUTO);
        }
    };

    Runnable liftDownSubmersibleRunnable = new Runnable() {
        @Override
        public void run() {
            lift.setTarget(ANGLE_ZERO + 3);
        }
    };

    Runnable liftDownRunnable = new Runnable() {
        @Override
        public void run() {
            lift.setTarget(ANGLE_ZERO);
        }
    };

    Runnable liftDownDefaultRunnable = new Runnable() {
        @Override
        public void run() {
            lift.setTarget(ANGLE_DEFAULT);
        }
    };

    Runnable intakeUpRunnable = new Runnable() {
        @Override
        public void run() {
            intake.score();
        }
    };

    Runnable spinVerticalRunnable = new Runnable() {
        @Override
        public void run() {
            intake.spinVertical();
        }
    };

    Runnable retractRunnable = new Runnable() {
        @Override
        public void run() {
            extend.setTarget(0);
        }
    };

    Runnable extendRunnable = new Runnable() {
        @Override
        public void run() {
            extend.setTarget(700);
        }
    };

    Runnable intakeDownRunnable = new Runnable() {
        @Override
        public void run() {
            intake.submersibleDown();
        }
    };

    Runnable intakeMidRunnable = new Runnable() {
        @Override
        public void run() {
            intake.submersibleMid();
        }
    };

    Runnable extend1Runnable = new Runnable() {
        @Override
        public void run() {
            extend.setTarget(130);
        }
    };

    Runnable extend2Runnable = new Runnable() {
        @Override
        public void run() {
            extend.setTarget(130);
        }
    };

    Runnable extend3Runnable = new Runnable() {
        @Override
        public void run() {
            // did some math, this should be enough (for test one not two)
            extend.setTarget(280);
        }
    };

    Runnable resetTimerRunnable = new Runnable() {
        @Override
        public void run() {
            actionTimer.resetTimer();
        }
    };

    // 0.8s
    public void grabFromSubmersible() {
        // make sure there is enough time to turn intake
        if (actionTimer.getElapsedTimeSeconds() > 0.15) {
            if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                lift.setTarget(ANGLE_ZERO);
            } else if (actionTimer.getElapsedTimeSeconds() < 0.5) {
                intake.close();
            } else if (actionTimer.getElapsedTimeSeconds() < 0.8) {
                lift.setTarget(ANGLE_ZERO + 15);
                intake.score();
            }
        }
    }

//    public void prepareScore(PathChain pChain, int index) {
//        pChain.setCallbacks(new ArrayList<PathCallback>(
//                new PathCallback(0.1, retractRunnable, 0, index),
//        ));
//    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        int x1 = incrementor.getValue(0);
        int x2 = incrementor.getValue(1);
        int x3 = incrementor.getValue(2);
        int x4 = incrementor.getValue(3);

        Pose submersible1Pose = new Pose(72-(2*x1), 50, Math.toRadians(90));
        Pose submersible2Pose = new Pose(72-(2*x2), 50, Math.toRadians(90));
        Pose submersible3Pose = new Pose(72-(2*x3), 50, Math.toRadians(90));
        Pose submersible4Pose = new Pose(72-(2*x4), 50, Math.toRadians(90));

        Point control1Point = new Point(72-(2*x1), 24, Point.CARTESIAN);
        Point control2Point = new Point(72-(2*x2), 24, Point.CARTESIAN);
        Point control3Point = new Point(72-(2*x3), 24, Point.CARTESIAN);
        Point control4Point = new Point(72-(2*x4), 24, Point.CARTESIAN);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, liftUpRunnable)
                .addParametricCallback(0.4, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.1, intakeMidRunnable)
                .addParametricCallback(0.3, retractRunnable)
                .addParametricCallback(0.5, liftDownRunnable)
                .addParametricCallback(0.7, extend1Runnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup1Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        grabPickup2= follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.1, intakeMidRunnable)
                .addParametricCallback(0.3, retractRunnable)
                .addParametricCallback(0.5, liftDownRunnable)
                .addParametricCallback(0.7, extend2Runnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup2Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose),
                                new Point(pickup3Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0.1, intakeMidRunnable)
                .addParametricCallback(0.3, retractRunnable)
                .addParametricCallback(0.5, liftDownRunnable)
                .addParametricCallback(0.7, extend3Runnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(pickup3Pose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        submersible1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                control1Point,
                                new Point(submersible1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersible1Pose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.4, liftDownSubmersibleRunnable)
                .addParametricCallback(0.6, intakeDownRunnable)
                .build();

        submersible2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                control2Point,
                                new Point(submersible2Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersible2Pose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.4, liftDownSubmersibleRunnable)
                .addParametricCallback(0.6, intakeDownRunnable)
                .build();

        submersible3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                control3Point,
                                new Point(submersible3Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersible3Pose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.4, liftDownSubmersibleRunnable)
                .addParametricCallback(0.6, intakeDownRunnable)
                .build();

        submersible4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                control4Point,
                                new Point(submersible4Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersible4Pose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.4, liftDownSubmersibleRunnable)
                .addParametricCallback(0.6, intakeDownRunnable)
                .build();

        scoreSubmersible1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible1Pose),
                                control1Point,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersible1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.98, resetTimerRunnable)
                .build();

        scoreSubmersible2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible2Pose),
                                control2Point,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersible2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        scoreSubmersible3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible3Pose),
                                control3Point,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersible3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

        scoreSubmersible4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible4Pose),
                                control4Point,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersible4Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, retractRunnable)
                .addParametricCallback(0.3, liftUpRunnable)
                .addParametricCallback(0.5, extendRunnable)
                .addParametricCallback(0.9, intakeUpRunnable)
                .addParametricCallback(0.99, resetTimerRunnable)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
                // TODO: first one may be more precise because conditions are different but check this
            case 1:
                if(!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.open();
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                    } else if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                        intake.close();
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                        intake.submersibleDown();
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.open();
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(scorePickup2, true);
                        setPathState(5);
                    } else if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                        intake.close();
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                        intake.submersibleDown();
                    }
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.open();
                        follower.followPath(grabPickup3, true);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                    } else if (actionTimer.getElapsedTimeSeconds() > 1.5) {
                        intake.close();
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.8) {
                        intake.submersibleDown();
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.open();
                        follower.followPath(submersible1, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9:
                if (yellowPipeline.isBlockDetected()) {
                    if (yellowPipeline.getOrientation() == 1) {
                        intake.spinHorizontal();
                    } else {
                        intake.spinVertical();
                    }
                    actionTimer.resetTimer();
                    extend.setTarget(extend.getCurrentPosition());
                    setPathState(10);
                } else {
                    if (extend.getCurrentPosition() > 600) {
                        extend.setTarget(extend.getCurrentPosition());
                        actionTimer.resetTimer();
                        setPathState(10);
                    }
                    extend.manual(1);
                }
                break;
            case 10:
                grabFromSubmersible();
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    extend.setTarget(0);
                    follower.followPath(scoreSubmersible1, true);
                    actionTimer.resetTimer();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        intake.open();
                        follower.followPath(submersible2, true);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                if (yellowPipeline.isBlockDetected()) {
                    if (yellowPipeline.getOrientation() == 1) {
                        intake.spinHorizontal();
                    } else {
                        intake.spinVertical();
                    }
                    actionTimer.resetTimer();
                    extend.setTarget(extend.getCurrentPosition());
                    setPathState(14);
                } else {
                    if (extend.getCurrentPosition() > 600) {
                        extend.setTarget(extend.getCurrentPosition());
                        actionTimer.resetTimer();
                        setPathState(14);
                    }
                    extend.manual(1);
                }
                break;
            case 14:
                grabFromSubmersible();
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    extend.setTarget(0);
                    follower.followPath(scoreSubmersible2, true);
                    actionTimer.resetTimer();
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        intake.open();
                        follower.followPath(submersible3, true);
                        setPathState(16);
                    }
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(17);
                }
                break;
            case 17:
                if (yellowPipeline.isBlockDetected()) {
                    if (yellowPipeline.getOrientation() == 1) {
                        intake.spinHorizontal();
                    } else {
                        intake.spinVertical();
                    }
                    actionTimer.resetTimer();
                    extend.setTarget(extend.getCurrentPosition());
                    setPathState(18);
                } else {
                    if (extend.getCurrentPosition() > 600) {
                        extend.setTarget(extend.getCurrentPosition());
                        actionTimer.resetTimer();
                        setPathState(18);
                    }
                    extend.manual(1);
                }
                break;
            case 18:
                grabFromSubmersible();
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    extend.setTarget(0);
                    follower.followPath(scoreSubmersible3, true);
                    actionTimer.resetTimer();
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    if (actionTimer.getElapsedTimeSeconds() > 0.1) {
                        intake.open();
                        follower.followPath(submersible4, true);
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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();
        autonomousPathUpdate();

        lift.loop();
        extend.loop();

        double currentTime = loopTimer.getElapsedTimeSeconds();
        loopTimes.add(currentTime);
        loopTimer.resetTimer();

        // Feedback to Driver Hub
        telemetry.addData("Busy", follower.isBusy());
//        telemetry.addData("Detected", yellowPipeline.isBlockDetected());
//        telemetry.addData("Orientation", yellowPipeline.getOrientation());
        telemetry.addData("Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("Loop Time", currentTime);
        // TODO: CHECK THESE IN LOGCAT
        System.out.println(currentTime);
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        loopTimer = new Timer();
        loopTimes = new ArrayList<>();

        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");

        intake = new Claw(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // i need to instantiate all motors before lynx hub stuff
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        lift = new TeleOpLift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);

        // TODO: Experiment with different manual power speeds to make it faster
        // extend.setManualPower(0.7);

        yellowPipeline = new YellowRedPipeline();

        // as we are driving from the sub perhaps we want to enable this


//        myVisionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(myAprilTagProcessor)
//                .setCameraResolution(new Size(640, 480))
//                // NOW I CAN USE THE FAST FORMAT 90 fps yay
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableCameraMonitoring(true)
//                .setAutoStopLiveView(true)
//                .build();
//
//        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.setPipeline(yellowPipeline);
//                // TODO: MIGHT BE SIDEWAYS right?
//                // phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
        // phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                telemetry.addLine("Error");
//            }
//        });
    }

    /** Driver selects four positions to go to. **/
    @Override
    public void init_loop() {
        if (gamepad1.dpad_left && !prevDpadLeft) {
            incrementor.decrementIndex();
        } else if (gamepad1.dpad_right && !prevDpadRight) {
            incrementor.incrementIndex();
        } else if (gamepad1.dpad_up && !prevDpadUp) {
            incrementor.incrementValue();
        } else if (gamepad1.dpad_down && !prevDpadDown) {
            incrementor.decrementValue();
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadDown = gamepad1.dpad_down;

        telemetry.addData("x1", incrementor.getValue(0));
        telemetry.addData("x2", incrementor.getValue(1));
        telemetry.addData("x3", incrementor.getValue(2));
        telemetry.addData("x4", incrementor.getValue(3));
        telemetry.addData("editing", incrementor.getCurrentIndex()+1);
        telemetry.update();

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        loopTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        double avg = 0;
        for (double x: loopTimes) {
            avg += x;
        }
        avg = avg/loopTimes.size();

        telemetry.addData("average loop time", avg);
    }
}


