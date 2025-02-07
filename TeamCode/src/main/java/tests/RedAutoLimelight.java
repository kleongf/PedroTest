package tests;

import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// this heuristic will just move until it hits the center of x and y. it moves a maximum of 12 inches
// in the x direction and 8 in the y direction.
// we will use the python snapscript to get the best one
@Autonomous(name = "Autonomous Red Limelight Testing")
public class RedAutoLimelight extends OpMode {
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

    public Limelight3A limelight;

    /** Start Pose of our robot */
    final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));

    // gonna have to change these
    final Pose submersibleStartPose = new Pose(84, 38, Math.toRadians(90));
    final Pose submersibleXPose = new Pose(70, 38, Math.toRadians(90));

    PathChain goToSubmersible, trimX, trimY, scorePickup;

    Lift lift;
    Extend extend;
    Intake intake;

    //                    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
//                        lift.setTarget(ANGLE_MID);
//                    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
//                        extend.setTarget(EXTEND_MID);
//                    } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
//                        intake.IntakeForward();
//                    } else if (actionTimer.getElapsedTimeSeconds() < 1.2) {
//                        intake.IntakeDown();
//                    }
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    if (actionTimer.getElapsedTimeSeconds() > 1.5) {
//                        // changed holdEnd to false
//                        follower.followPath(grabPickup,false);
//                        setPathState(2);
//                    }
//    if (actionTimer.getElapsedTimeSeconds() < 0.3) {
//        intake.IntakeUp();
//    } else if (actionTimer.getElapsedTimeSeconds() < 0.6) {
//        lift.setTarget(ANGLE_ZERO);
//    } else if (actionTimer.getElapsedTimeSeconds() < 0.9) {
//        extend.setTarget(EXTEND_ZERO);
//    }
//                    if (actionTimer.getElapsedTimeSeconds() > 1.2) {
//        follower.followPath(scorePickup, true);
//        setPathState(3);
//    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        goToSubmersible = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(80, 22, Point.CARTESIAN),
                                new Point(submersibleStartPose)
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), submersibleStartPose.getHeading())
                .build();

        trimX = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(submersibleStartPose),
                                new Point(submersibleXPose)
                        )
                )
                .setLinearHeadingInterpolation(submersibleStartPose.getHeading(), submersibleXPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(goToSubmersible, false);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(trimX, false);
                    setPathState(2);
                }
            case 2:
                if (follower.isBusy()) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        double[] pythonOutput = result.getPythonOutput();
                        telemetry.addData("PythonOutput", pythonOutput);
                        // pythonOutput[0] is the x position of the target on the frame
                        // our arm is going to be calibrated to the exact center of the camera (or offset a bit)
                        if (pythonOutput[0] != -1 && pythonOutput[1] != -1) {
                            if (Math.abs(pythonOutput[0] - 160) < 10) {
                                follower.breakFollowing();
                            }
                            Pose currentPose = follower.getPose();
                            trimY = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    new Point(currentPose),
                                                    new Point(new Pose(
                                                            currentPose.getX(),
                                                            currentPose.getY() + 10,
                                                            currentPose.getHeading()
                                                    ))
                                            )
                                    )
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), currentPose.getHeading())
                                    .build();
                            actionTimer.resetTimer();
                            follower.setMaxPower(0.5);
                            follower.followPath(trimY, false);
                            setPathState(3);
                        }
                    }
                } else {
                    Pose currentPose = follower.getPose();
                    trimY = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose),
                                            new Point(new Pose(
                                                    currentPose.getX(),
                                                    currentPose.getY() + 10,
                                                    currentPose.getHeading()
                                            ))
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), currentPose.getHeading())
                            .build();
                    follower.followPath(trimY);
                    setPathState(3);
                }
                break;
            case 3:
                if (follower.isBusy()) {
                    LLResult result = limelight.getLatestResult();
                    if (result != null) {
                        double[] pythonOutput = result.getPythonOutput();
                        telemetry.addData("PythonOutput", pythonOutput);
                        // pythonOutput[1] is the y position of the target on the frame
                        // our arm is going to be calibrated to the exact center of the camera (or offset a bit)
                        if (pythonOutput[0] != -1 && pythonOutput[1] != -1) {
                            if (Math.abs(pythonOutput[1] - 120) < 8) {
                                follower.breakFollowing();
                            }
                            Pose currentPose = follower.getPose();
                            scorePickup = follower.pathBuilder()
                                    .addPath(
                                            new BezierLine(
                                                    new Point(currentPose),
                                                    new Point(scorePose)
                                            )
                                    )
                                    .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                                    .build();
                            actionTimer.resetTimer();
                            follower.setMaxPower(1);
                            follower.followPath(scorePickup, true);
                            setPathState(4);
                        }
                    }
                } else {
                    Pose currentPose = follower.getPose();
                    scorePickup = follower.pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            new Point(currentPose),
                                            new Point(scorePose)
                                    )
                            )
                            .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                            .build();
                    actionTimer.resetTimer();
                    follower.setMaxPower(1);
                    follower.followPath(scorePickup, true);
                    setPathState(4);
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();
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

