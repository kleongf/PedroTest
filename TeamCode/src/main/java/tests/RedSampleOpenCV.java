package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import shared.Extend;
import shared.Intake;
import shared.Lift;
import vision.YellowPipeline;

@Autonomous(name = "Autonomous Red OpenCV Testing New")
public class RedSampleOpenCV extends OpMode {
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
    private double ticksPerInch = 29;

    OpenCvCamera phoneCam;
    YellowPipeline yellowPipeline;

    /** Start Pose of our robot */
    final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    // gonna have to change these
    final Pose submersibleStartPose = new Pose(72, 30, Math.toRadians(90));

    final Pose submersibleEndPose = new Pose(72, 52, Math.toRadians(90));

    PathChain goToSubmersible, trimY, scorePickup;

    Lift lift;
    Extend extend;
    Intake intake;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
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
                extend.setTarget(12);
                follower.followPath(goToSubmersible, false);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    extend.setTarget(550);
                    // slowing it down
                    follower.setMaxPower(0.36);
                    follower.followPath(trimY, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (follower.isBusy()) {
                    telemetry.addData("Detected", yellowPipeline.isBlockDetected());
                    if (yellowPipeline.isBlockDetected()) {
                        lift.setTarget(9);
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
                        follower.setMaxPower(1);
                        actionTimer.resetTimer();
                        follower.followPath(scorePickup);
                        setPathState(3);
                    }
                } else {
                    actionTimer.resetTimer();
                    lift.setTarget(9);
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
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup);
                    actionTimer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.isBusy()) {
                    if (actionTimer.getElapsedTime() < 0.3) {
                        lift.setTarget(11);
                    } else if (actionTimer.getElapsedTime() < 0.8) {
                        extend.setTarget(0);
                    }
                } else {
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

        telemetry.addData("Busy", follower.isBusy());
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



