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
import vision.YellowRedPipeline;

import static shared.Constants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous Vision Test")
public class VisionTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    private final Pose submersibleStartPose = new Pose(80, 48, Math.toRadians(90));

    private PathChain submersible1, submersible2, submersible3, submersible4, scoreSubmersible1, scoreSubmersible2, scoreSubmersible3, scoreSubmersible4, goToSubmersible, scorePickup;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public AnalogInput analogEncoder;

    Lift lift;
    Extend extend;
    Claw intake;

    OpenCvCamera phoneCam;
    YellowRedPipeline yellowPipeline;

    int x1 = 0;
    int x2 = 0;
    int x3 = 0;
    int x4 = 0;

    int index = 0;

    boolean prevDpadUp = false;
    boolean prevDpadDown = false;
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;

    public void incrementValue(int d) {
        switch (index) {
            case 0:
                x1 += d;
            case 1:
                x2 += d;
            case 2:
                x3 += d;
            case 3:
                x4 += d;
        }
    }

    public void incrementIndex() {
        if (index < 3) {
            index++;
        }
    }

    public void decrementIndex() {
        if (index > 0) {
            index--;
        }
    }

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

    // 0.8s
    public void prepareScore() {
        if (actionTimer.getElapsedTimeSeconds() < 0.2) {
            // retract (from grabbing)
            extend.setTarget(0);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.4) {
            // lift up and flip up intake
            lift.setTarget(ANGLE_UP_AUTO);
            intake.spinVertical();
        } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
            // extend up
            extend.setTarget(EXTEND_HIGH_AUTO);
        } else if (actionTimer.getElapsedTimeSeconds() < 0.8) {
            // scoring position
            intake.score();
        }
    }

    // 1.0s
    public void grabFromSubmersible() {
        // make sure there is enough time to turn intake
        if (actionTimer.getElapsedTimeSeconds() > 0.15) {
            if (actionTimer.getElapsedTimeSeconds() < 0.3) {
                lift.setTarget(9 - Math.toDegrees(Math.atan(1 / (12 + (extend.getCurrentPosition() / 29.0)))));
            } else if (actionTimer.getElapsedTimeSeconds() < 0.5) {
                intake.close();
            } else if (actionTimer.getElapsedTimeSeconds() < 0.8) {
                // need the extra push
                lift.setTarget(ANGLE_ZERO + 5);
            } else if (actionTimer.getElapsedTimeSeconds() < 1) {
                intake.score();
            }
        }
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        Pose submersible1Pose = new Pose(72-(2*x1), 48, Math.toRadians(90));
        Pose submersible2Pose = new Pose(72-(2*x2), 48, Math.toRadians(90));
        Pose submersible3Pose = new Pose(72-(2*x3), 48, Math.toRadians(90));
        Pose submersible4Pose = new Pose(72-(2*x4), 48, Math.toRadians(90));

        Point control1Point = new Point(72-(2*x1), 24, Point.CARTESIAN);
        Point control2Point = new Point(72-(2*x2), 24, Point.CARTESIAN);
        Point control3Point = new Point(72-(2*x3), 24, Point.CARTESIAN);
        Point control4Point = new Point(72-(2*x4), 24, Point.CARTESIAN);


        submersible1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                control1Point,
                                new Point(submersible1Pose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersible1Pose.getHeading())
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
                .build();

        submersible4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersible4Pose),
                                control4Point,
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersible4Pose.getHeading(), scorePose.getHeading())
                .build();

        goToSubmersible = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose),
                                new Point(75, 23, Point.CARTESIAN),
                                new Point(submersibleStartPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersibleStartPose.getHeading())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersibleStartPose),
                                new Point(75, 23, Point.CARTESIAN),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersibleStartPose.getHeading(), scorePose.getHeading())
                .build();


    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 7:
                intake.open();
                follower.followPath(submersible1, true);
                actionTimer.resetTimer();
                setPathState(8);
                break;
            case 8:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(9);
                } else {
                    lift.setTarget(ANGLE_ZERO + 3);
                    extend.setTarget(0);
                    intake.submersibleDown();
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
                    if (extend.getCurrentPosition() > 680) {
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
                if (follower.isBusy()) {prepareScore();}
                if (!follower.isBusy()) {
                    intake.open();
                    actionTimer.resetTimer();
                    follower.followPath(submersible2, true);
                    actionTimer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(13);
                } else {
                    lift.setTarget(ANGLE_ZERO + 3);
                    extend.setTarget(0);
                    intake.submersibleDown();
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
                    if (extend.getCurrentPosition() > 680) {
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
                if (follower.isBusy()) {prepareScore();}
                if (!follower.isBusy()) {
                    intake.open();
                    actionTimer.resetTimer();
                    follower.followPath(submersible3, true);
                    actionTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    extend.manual(1);
                    actionTimer.resetTimer();
                    setPathState(17);
                } else {
                    lift.setTarget(ANGLE_ZERO+3);
                    extend.setTarget(0);
                    intake.submersibleDown();
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
                    if (extend.getCurrentPosition() > 680) {
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
        follower.update();
        autonomousPathUpdate();

        lift.loop();
        extend.loop();

        // Feedback to Driver Hub
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Detected", yellowPipeline.isBlockDetected());
        telemetry.addData("Orientation", yellowPipeline.getOrientation());
        telemetry.addData("Timer", actionTimer.getElapsedTimeSeconds());
        telemetry.addData("pivot power", lift.getPower());
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


        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder, extendMotorTwo);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Claw(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        yellowPipeline = new YellowRedPipeline();
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
    public void init_loop() {
        if (gamepad1.dpad_left && !prevDpadLeft) {
            decrementIndex();
        } else if (gamepad1.dpad_right && !prevDpadRight) {
            incrementIndex();
        } else if (gamepad1.dpad_up && !prevDpadUp) {
            incrementValue(1);
        } else if (gamepad1.dpad_down && !prevDpadDown) {
            incrementValue(-1);
        }

        prevDpadUp = gamepad1.dpad_up;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadDown = gamepad1.dpad_down;

        telemetry.addData("x1", x1);
        telemetry.addData("x2", x2);
        telemetry.addData("x3", x3);
        telemetry.addData("x4", x4);
        telemetry.addData("editing", index+1);
        telemetry.update();

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(7);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

