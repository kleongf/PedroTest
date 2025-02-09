//package teleop;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//
//import shared.Claw;
//import shared.Extend;
//import shared.Lift;
//
//public class Teleop {
//
//    private Extend extend;
//    private Lift lift;
//
//    private Claw intake;
//    private Claw.GrabState intakeGrabState;
//    private Claw.PivotState intakePivotState;
//    private Claw.RotateState intakeRotateState;
//
//    private Follower follower;
//    private Pose startPose;
//
//    private Telemetry telemetry;
//
//    private Gamepad gamepad1, gamepad2;
//    private Gamepad currentGamepad1 = new Gamepad(), currentGamepad2 = new Gamepad(), previousGamepad1 = new Gamepad(), previousGamepad2 = new Gamepad();
//
//    private Timer autoBucketTimer = new Timer(), transferTimer = new Timer(), submersibleTimer = new Timer(), opmodeTimer = new Timer();
//
//    private int flip = 1, autoBucketState = -1, transferState = -1, submersibleState = -1, autoSpecimenState = -1;
//
//    public double speed = 0.75;
//
//    private boolean fieldCentric, actionBusy, driveBusy, transferSampleDetected = false;
//
//    private PathChain autoBucketTo, autoBucketBack;
//    private Pose autoBucketToEndPose, autoBucketBackEndPose;
//
//
//    public Teleop(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose startPose, Gamepad gamepad1, Gamepad gamepad2) {
//        lift = new Lift(hardwareMap);
//        extend = new Extend(hardwareMap);
//        intake = new Claw(hardwareMap, intakeGrabState, intakePivotState, intakeRotateState);
//
//        this.follower = follower;
//        this.startPose = startPose;
//
//        this.telemetry = telemetry;
//        this.gamepad1 = gamepad1;
//        this.gamepad2 = gamepad2;
//    }
//
//    public void init() {
//        follower.setStartingPose(startPose);
//        follower.setPose(startPose);
//    }
//
//    public void start() {
//        follower.startTeleopDrive();
//
//        opmodeTimer.resetTimer();
//    }
//
//    public void update() {
//        if (actionNotBusy()) {
//            previousGamepad1.copy(currentGamepad1);
//            previousGamepad2.copy(currentGamepad2);
//            currentGamepad1.copy(gamepad1);
//            currentGamepad2.copy(gamepad2);
//
//            speed = (gamepad2.left_trigger > 0.5) ? 0.5 : 1;
//
//            if (gamepad1.cross && !previousGamepad1.cross)
//                extend.();
//
//            if (gamepad1.left_trigger > 0.1)
//                extend.toZero();
//
//            if (currentGamepad2.a && !previousGamepad2.a)
//                outtake.switchGrabState();
//
//            if (currentGamepad2.y && !previousGamepad2.y) {
//                extend.setLimitToSample();
//                outtake.transferDetected();
//                intake.hover();
//            }
//
//            if (currentGamepad2.x && !previousGamepad2.x) {
//                extend.setLimitToSample();
//                outtake.score();
//                intake.hover();
//            }
//
//            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
//                specimenGrabPos();
//
//            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
//                specimenScorePos();
//
//            if (currentGamepad2.b && !previousGamepad2.b)
//                startTransfer();
//
//            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
//                intake.switchGrabState();
//            }
//
//            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
//                startSubmersible();
//                outtake.score();
//            }
//
//            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
//                intake.rotateCycle(true);
//            }
//
//            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
//                intake.rotateCycle(false);
//            }
//
//            if (gamepad2.left_stick_button) {
//                outtake.hang();
//                intake.transferDetected();
//                extend.toZero();
//            }
//
//            if (gamepad2.right_stick_button) {
//                intake.transferDetected();
//            }
//
//            if (!driveBusy()) {
//                follower.setTeleOpMovementVectors(flip * -gamepad1.left_stick_y * speed, flip * -gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed * 0.5, !fieldCentric);
//            }
//
//            if(gamepad1.dpad_right) {
//                stopActions();
//            }
//
//            if(gamepad1.y) {
//                startAutoSpecimen();
//            }
//
//        } else {
//            if(gamepad1.dpad_right) {
//                stopActions();
//            }
//        }
//
//        lift.updatePIDF();
//        outtake.loop();
//
//        autoBucket();
//        transfer();
//        submersible();
//        autoSpecimen();
//
//
//        follower.update();
//        telemetry.addData("X: ", follower.getPose().getX());
//        telemetry.addData("Y: ", follower.getPose().getY());
//        telemetry.addData("Heading: ", follower.getPose().getHeading());
//        telemetry.addData("Action Busy?: ", actionBusy);
//        telemetry.addData("Drive Busy? ", driveBusy);
//        telemetry.addData("Auto Bucket State", autoBucketState);
//        telemetry.addData("Transfer State", transferState);
//        telemetry.addData("Submersible State", submersibleState);
//        extend.telemetry();
//        lift.telemetry();
//        outtake.telemetry();
//        intake.telemetry();
//        // vision.telemetry();
//        telemetry.update();
//    }
//
//    private void specimenGrabPos() {
//        extend.setLimitToSpecimen();
//        outtake.startSpecGrab();
//        intake.specimen();
//    }
//
//    private void specimenScorePos() {
//        extend.setLimitToSpecimen();
//        outtake.specimenScore();
//        intake.specimen();
//    }
//
//    private void retract() {
//        s
//    }
//
//    private void transfer() {
//        switch (transferState) {
//            case 1:
//                intake.close();
//
//                //     transferSampleDetected = (intake.getColor() == IntakeColor.BLUE || intake.getColor() == IntakeColor.RED || intake.getColor() == IntakeColor.YELLOW);
//                outtake.transferUndetected();
//                intake.transferUndetected();
//
//
//                setTransferState(2);
//                break;
//            case 2:
//                if (transferTimer.getElapsedTimeSeconds() > 0.1) {
//
//                    outtake.setRotateState(OuttakeSubsystem.RotateState.TRANSFER_DETECTED);
//
//                    extend.toTransfer();
//                    setTransferState(3);
//                }
//                break;
//            case 3:
//                if (transferTimer.getElapsedTimeSeconds() > 0.15) {
//                    outtake.transferDetected();
//
//
//                    setTransferState(4);
//                }
//                break;
//            case 4:
//                if (transferTimer.getElapsedTimeSeconds() > 0) {
//                    outtake.close();
//                    setTransferState(5);
//                }
//                break;
//            case 5:
//                if (transferTimer.getElapsedTimeSeconds() > 0.5) {
//                    outtake.score();
//                    setTransferState(6);
//                }
//                break;
//            case 6:
//                if (transferTimer.getElapsedTimeSeconds() > 0) {
//                    intake.open();
//                    setTransferState(7);
//                }
//                break;
//            case 7:
//                if (transferTimer.getElapsedTimeSeconds() > 0.25) {
//                    intake.hover();
//                    actionBusy = false;
//                    transferSampleDetected = false;
//                    setTransferState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setTransferState(int x) {
//        transferState = x;
//        transferTimer.resetTimer();
//    }
//
//    public void startTransfer() {
//        setTransferState(1);
//    }
//
//    private void submersible() {
//        switch (submersibleState) {
//            case 0:
//                intake.ground();
//                intake.open();
//                outtake.transferDetected();
//                setSubmersibleState(1);
//                break;
//            case 1:
//                if(submersibleTimer.getElapsedTimeSeconds() > 0.3) {
//                    intake.close();
//                    setSubmersibleState(2);
//                }
//                break;
//            case 2:
//                if (submersibleTimer.getElapsedTimeSeconds() > 0.25) {
//                    intake.hover();
//                    setSubmersibleState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setSubmersibleState(int x) {
//        submersibleState = x;
//        submersibleTimer.resetTimer();
//    }
//
//    public void startSubmersible() {
//        setSubmersibleState(0);
//    }
//
//    private void autoBucket() {
//        switch (autoBucketState) {
//            case 1:
//                actionBusy = true;
//                outtake.open();
//                outtake.transferDetected();
//                extend.toZero();
//
//                follower.breakFollowing();
//                follower.setMaxPower(0.85);
//
//                autoBucketToEndPose = new Pose(17.750, 125.500, Math.toRadians(-45));
//
//                autoBucketTo = follower.pathBuilder()
//                        .addPath(new BezierCurve(new Point(follower.getPose()), new Point(58.000, 119.000, Point.CARTESIAN), new Point(autoBucketToEndPose)))
//                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
//                        .build();
//
//                follower.followPath(autoBucketTo, true);
//
//                setAutoBucketState(2);
//                break;
//            case 2:
//                if (autoBucketTimer.getElapsedTimeSeconds() > 2) {
//                    outtake.close();
//                    setAutoBucketState(3);
//                }
//                break;
//            case 3:
//                if (autoBucketTimer.getElapsedTimeSeconds() > 0.5) {
//                    lift.toHighBucket();
//                    setAutoBucketState(4);
//                }
//                break;
//            case 4:
//                if (autoBucketTimer.getElapsedTimeSeconds() > 0.5) {
//                    outtake.score();
//                    setAutoBucketState(5);
//                }
//                break;
//            case 5:
//                if (((follower.getPose().getX() <  autoBucketToEndPose.getX() + 0.5) && (follower.getPose().getY() > autoBucketToEndPose.getY() - 0.5)) && (lift.getPos() > RobotConstants.liftToHighBucket - 50) && autoBucketTimer.getElapsedTimeSeconds() > 1) {
//                    outtake.open();
//                    setAutoBucketState(9);
//                    //setAutoBucketState(6);
//                }
//                break;
//            case 6:
//                if(autoBucketTimer.getElapsedTimeSeconds() > 0.5) {
//                    autoBucketBackEndPose = new Pose(60, 104, Math.toRadians(270));
//
//                    autoBucketBack = follower.pathBuilder()
//                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(58.000, 119.000, Point.CARTESIAN), new Point(autoBucketBackEndPose)))
//                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), autoBucketToEndPose.getHeading())
//                            .build();
//
//                    follower.followPath(autoBucketBack, true);
//
//                    outtake.open();
//                    outtake.transferDetected();
//                    setAutoBucketState(7);
//                }
//                break;
//            case 7:
//                if(autoBucketTimer.getElapsedTimeSeconds() > 0.5) {
//                    lift.toZero();
//                    extend.toFull();
//                    setAutoBucketState(8);
//                }
//                break;
//            case 8:
//                if((follower.getPose().getX() >  autoBucketBackEndPose.getX() - 0.5) && (follower.getPose().getY() < autoBucketBackEndPose.getY() + 0.5)) {
//                    intake.ground();
//                    setAutoBucketState(9);
//                }
//                break;
//            case 9:
//                follower.breakFollowing();
//                follower.setMaxPower(1);
//                follower.startTeleopDrive();
//                actionBusy = false;
//                setAutoBucketState(-1);
//                break;
//        }
//    }
//
//    public void setAutoBucketState(int x) {
//        autoBucketState = x;
//        autoBucketTimer.resetTimer();
//    }
//
//    public void startAutoBucket() {
//        setAutoBucketState(1);
//    }
//
//    public void autoSpecimen() {
//        switch (autoSpecimenState) {
//            case 1:
//                driveBusy = true;
//                PathChain path = follower.pathBuilder()
//                        .addPath(new BezierCurve(new Point(follower.getPose()), new Point(follower.getPose().getX() - 10, follower.getPose().getY(), Point.CARTESIAN), new Point(humanPlayerPose.getX() + 10, humanPlayerPose.getY(), Point.CARTESIAN), new Point(humanPlayerPose)))
//                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), humanPlayerPose.getHeading())
//                        .setZeroPowerAccelerationMultiplier(2)
//                        .build();
//                follower.followPath(path, true);
//                flip = -1;
//                setAutoSpecimenState(2);
//            case 2:
//                if(!follower.isBusy()) {
//                    driveBusy = false;
//                    setAutoSpecimenState(-1);
//                }
//        }
//    }
//
//    public void setAutoSpecimenState(int state) {
//        autoSpecimenState = state;
//    }
//
//    public void startAutoSpecimen() {
//        setAutoSpecimenState(1);
//    }
//
//    private boolean actionNotBusy() {
//        return !actionBusy;
//    }
//
//    private boolean driveBusy() {
//        return driveBusy;
//    }
//
//    private void stopActions() {
//        follower.breakFollowing();
//        follower.setMaxPower(1);
//        follower.startTeleopDrive();
//        actionBusy = false;
//        driveBusy = false;
//        setTransferState(-1);
//        setAutoBucketState(-1);
//        setAutoSpecimenState(-1);
//    }
//}
