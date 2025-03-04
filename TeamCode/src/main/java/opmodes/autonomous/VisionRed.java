package opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import robot.Alliance;
import robot.OpModeType;
import robot.Robot;
import vision.YCrCbPipelineRY;
import vision.YellowRedPipeline;

@Autonomous(name = "Red Vision: 0+1")
public class VisionRed extends OpMode {
    public Robot robot;

    OpenCvCamera camera;
    YCrCbPipelineRY pipeline;

    private Follower follower;

    private Timer pathTimer;
    private int pathState;

    private final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));
    private final Pose scorePose = new Pose(123.5, 18.5, Math.toRadians(135));
    private final Pose submersiblePose = new Pose(72, 48, Math.toRadians(90));

    private PathChain submersible1, scoreSubmersible1;

    public void buildPaths() {
        submersible1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startPose),
                                new Point(submersiblePose)
                        )
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), submersiblePose.getHeading())
                .build();

        scoreSubmersible1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(submersiblePose),
                                new Point(scorePose)
                        )
                )
                .setLinearHeadingInterpolation(submersiblePose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // go to submersible
            case 0:
                follower.followPath(submersible1, true);
                robot.startSubmersible();
                setPathState(1);
                break;
            // extend
            case 1:
                if (!follower.isBusy() && !robot.isSubmersibleBusy()) {
                    robot.getExtend().manual(1);
                    setPathState(2);
                }
                break;
            // update orientation
            case 2:
                if (pipeline.isBlockDetected() || robot.getExtend().getCurrentPosition() > 600) {
                    if (pipeline.getOrientation() == 1) {
                        robot.getIntake().spinHorizontal();
                    } else {
                        robot.getIntake().spinVertical();
                    }
                    robot.holdExtendPosition();
                    setPathState(3);
                } else {
                    robot.getExtend().manual(1);
                }
                break;
            // grab
            case 3:
                robot.startGrab();
                setPathState(4);
                break;
            // drive back + prepare score
            case 4:
                if (!robot.isGrabBusy()) {
                    follower.followPath(scoreSubmersible1, true);
                    robot.startPrepare();
                    setPathState(5);
                }
                break;
            // deposit sample
            case 5:
                if (!follower.isBusy() && !robot.isPrepareBusy()) {
                    robot.startDeposit();
                    setPathState(6);
                }
                break;
            // wait lol
            case 6:
                if (!robot.isDepositBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions and reset the timers of switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        robot.loop();
        autonomousPathUpdate();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry, Alliance.RED, OpModeType.AUTONOMOUS, startPose);
        follower = robot.getFollower();
        pathTimer = new Timer();
        robot.setIsPickup(false);
        // TODO: Experiment with different manual power speeds to make it faster
        // extend.setManualPower(0.7);
        pipeline = new YCrCbPipelineRY();
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(pipeline);
                // TODO: MIGHT BE SIDEWAYS right?
                // phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error");
            }
        });

    }

    /**
     * Driver selects four positions to go to.
     **/
    @Override
    public void init_loop() {}

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        buildPaths();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {}
}

