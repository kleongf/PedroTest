package robot;

import static com.pedropathing.util.Constants.setConstants;

import static robot.RobotConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.subsystems.ExtendSubsystem;
import robot.subsystems.IntakeSubsystem;
import robot.subsystems.LiftSubsystem;
import shared.Incrementor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Robot {
    List<LynxModule> allHubs;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Gamepad gp1, gp2, pgp1, pgp2, cgp1, cgp2;
    private Alliance alliance;
    private Incrementor incrementor;
    private Follower follower;
    private ExtendSubsystem extend;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private OpModeType opModeType;
    public Pose scorePose;
    public Pose startPose;
    public double speed = 1;
    private boolean isPickup = false;

    /*
    timers for:

    use same timer
    DONE
    grabbing in teleop AND THE THREE PRELOADS:
        hold arm at a constant place: move servo down, close on release, move servo back up
        actually close on release is not that good of an idea
        we could also do this for the arm instead but i think its best not to
    grabbing in autonomous (different because of camera angle)
        arm down
        grab
        arm up


    also make getters for these states. this could be useful in auto so that we don't do
    something before a subsystem is done

    also make getters for subsystems so we can edit them directly in auto

    DONE
    depositing (intake up, open, intake mid)

    preparing score (retract, lift up, extend up)

    retracting from score (retract, lift down, extend to default)
    also use this in auto, just adjust paths accordingly
    overload method: just retract (useful in teleop)
    actually i wont need that just because preparescore takes care of it

    going to submersible (retract, lift down+some, intake down)

     */
    public Timer grabTimer, depositTimer, prepareTimer, retractTimer, submersibleTimer;
    public int grabState, depositState, prepareState, retractState, submersibleState;

    public Robot(HardwareMap hardwareMap, Gamepad gp1, Gamepad gp2, Telemetry telemetry, Alliance alliance, OpModeType opModeType, Pose startPose) {
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.pgp1 = new Gamepad();
        this.pgp2 = new Gamepad();
        this.cgp1 = new Gamepad();
        this.cgp2 = new Gamepad();
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.hardwareMap = hardwareMap;
        this.startPose = startPose;
        this.opModeType = opModeType;
        this.scorePose = new Pose(123.5, 18.5, Math.toRadians(135));
        incrementor = new Incrementor(4);

        // probably need a vision subsystem but idc rn
        // actually it should be fine, we'll keep it separate and only in the auto

        extend = new ExtendSubsystem(this.hardwareMap);
        lift = new LiftSubsystem(this.hardwareMap);
        intake = new IntakeSubsystem(this.hardwareMap);

        setConstants(FConstants.class, LConstants.class);
        follower = new Follower(this.hardwareMap);
        follower.setStartingPose(this.startPose);

        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        grabTimer = new Timer();
        depositTimer = new Timer();
        prepareTimer = new Timer();
        retractTimer = new Timer();
        submersibleTimer = new Timer();
    }

    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        grab();
        deposit();
        prepareScore();
        retract();
        submersible();

        extend.loop();
        lift.loop();
        follower.update();
        telemetry.update();
    }

    public void start() {
        if (opModeType == OpModeType.TELEOP) {
            follower.startTeleopDrive();
        }
    }

    public void grab() {
        if (opModeType == OpModeType.TELEOP || isPickup) {
            switch (grabState) {
                case 1:
                    intake.down();
                    setGrabState(2);
                    break;
                case 2:
                    if (grabTimer.getElapsedTimeSeconds() > 0.2) {
                        intake.close();
                        setGrabState(3);
                    }
                    break;
                case 3:
                    if (grabTimer.getElapsedTimeSeconds() > 0.2) {
                        intake.mid();
                        setGrabState(4);
                    }
                    break;
                case 4:
                    if (grabTimer.getElapsedTimeSeconds() > 0.2) {
                        setGrabState(-1);
                    }
            }
        } else {
            switch (grabState) {
                case 1:
                    lift.setTarget(LIFT_DEFAULT);
                    setGrabState(2);
                    break;
                case 2:
                    if (grabTimer.getElapsedTimeSeconds() > 0.4) {
                        intake.close();
                        setGrabState(3);
                    }
                    break;
                case 3:
                    if (grabTimer.getElapsedTimeSeconds() > 0.2) {
                        lift.setTarget(LIFT_SUBMERSIBLE);
                        setGrabState(4);
                    }
                    break;
                case 4:
                    if (grabTimer.getElapsedTimeSeconds() > 0.4) {
                        setGrabState(-1);
                    }
            }
        }
    }

    public void setIsPickup(boolean x) {
        isPickup = x;
    }

    public void setGrabState(int x) {
        grabState = x;
        grabTimer.resetTimer();
    }

    public void startGrab() {
        setGrabState(1);
    }

    public boolean isGrabBusy() {
        return grabState != -1;
    }

    public void deposit() {
            switch (depositState) {
                case 1:
                    intake.up();
                    setDepositState(2);
                    break;
                case 2:
                    if (depositTimer.getElapsedTimeSeconds() > 0.2) {
                        intake.open();
                        setDepositState(3);
                    }
                    break;
                case 3:
                    if (depositTimer.getElapsedTimeSeconds() > 0.2) {
                        intake.mid();
                        setDepositState(4);
                    }
                case 4:
                    if (depositTimer.getElapsedTimeSeconds() > 0.2) {
                        setDepositState(-1);
                    }
                    break;
            }
    }

    public void setDepositState(int x) {
        depositState = x;
        depositTimer.resetTimer();
    }

    public void startDeposit() {
        setDepositState(1);
    }

    public boolean isDepositBusy() {
        return depositState != -1;
    }

    // retract, lift up, extend up
    public void prepareScore() {
        switch (prepareState) {
            case 1:
                extend.setTarget(EXTEND_ZERO);
                setPrepareState(2);
                break;
            case 2:
                if (prepareTimer.getElapsedTimeSeconds() > 0.2) {
                    lift.setTarget(LIFT_UP);
                    setPrepareState(3);
                }
                break;
            case 3:
                if (prepareTimer.getElapsedTimeSeconds() > 0.2) {
                    extend.setTarget(EXTEND_MAX);
                    setPrepareState(4);
                }
                break;
            case 4:
                if (prepareTimer.getElapsedTimeSeconds() > 0.2) {
                    setPrepareState(-1);
                }
        }
    }

    public void setPrepareState(int x) {
        prepareState = x;
        prepareTimer.resetTimer();
    }

    public void startPrepare() {
        setPrepareState(1);
    }

    public boolean isPrepareBusy() {
        return prepareState != -1;
    }

    public void retract() {
        switch (retractState) {
            case 1:
                extend.setTarget(EXTEND_ZERO);
                setRetractState(2);
                break;
            case 2:
                if (retractTimer.getElapsedTimeSeconds() > 0.4) {
                    lift.setTarget(LIFT_DEFAULT);
                    setRetractState(3);
                }
                break;
            case 3:
                if (retractTimer.getElapsedTimeSeconds() > 0.4) {
                    extend.setTarget(EXTEND_DEFAULT);
                    setRetractState(4);
                }
                break;
            case 4:
                if (retractTimer.getElapsedTimeSeconds() > 0.2) {
                    setRetractState(-1);
                }
        }
    }

    public void setRetractState(int x) {
        retractState = x;
        retractTimer.resetTimer();
    }

    public void startRetract() {
        setRetractState(1);
    }

    public boolean isRetractBusy() {
        return retractState != -1;
    }

    // going to submersible (retract, lift down+some, intake down)
    // this is only really useful in auto

    public void submersible() {
        switch (submersibleState) {
            case 1:
                extend.setTarget(EXTEND_ZERO);
                setSubmersibleState(2);
                break;
            case 2:
                if (submersibleTimer.getElapsedTimeSeconds() > 0.2) {
                    lift.setTarget(LIFT_SUBMERSIBLE);
                    setSubmersibleState(3);
                }
                break;
            case 3:
                if (submersibleTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.down();
                    setSubmersibleState(4);
                }
                break;
            case 4:
                if (submersibleTimer.getElapsedTimeSeconds() > 0.2) {
                    setSubmersibleState(-1);
                }
        }
    }

    public void setSubmersibleState(int x) {
        retractState = x;
        retractTimer.resetTimer();
    }

    public void startSubmersible() {
        setRetractState(1);
    }

    public boolean isSubmersibleBusy() {
        return submersibleState != -1;
    }

    public void holdExtendPosition() {
        extend.setTarget(extend.getCurrentPosition());
    }

    public ExtendSubsystem getExtend() {
        return extend;
    }

    public IntakeSubsystem getIntake() {
        return intake;
    }

    public LiftSubsystem getLift() {
        return lift;
    }

    public Follower getFollower() {
        return follower;
    }

    public Incrementor getIncrementor() {
        return incrementor;
    }

    // update all controls
    public void updateControls() {
        // TODO: buttons for hang and also new robot for specimen
        pgp1.copy(cgp1);
        pgp2.copy(gp2);
        cgp1.copy(gp1);
        cgp2.copy(gp2);

        if (opModeType == OpModeType.TELEOP) {
            // right trigger: retract
            if (gp1.right_trigger > 0.5 && pgp1.right_trigger < 0.5) {
                startRetract();
            }
            // right bumper: prepare score
            if (gp1.right_bumper && !pgp1.right_bumper) {
                startPrepare();
            }

            // left stick: intake
            if (gp1.right_stick_button && !pgp1.right_stick_button) {
                startGrab();
            }
            // right stick: outtake
            if (gp1.left_stick_button && !pgp1.left_stick_button) {
                startDeposit();
            }

            // left trigger: extend to max
            if (gp1.left_trigger > 0.5 && pgp1.left_trigger < 0.5) {
                intake.toggleSpin();
            }

            // left bumper: toggle spin
            if (gp1.left_bumper && !pgp1.left_bumper) {
                extend.setTarget(EXTEND_MAX);
            }

            /* GAMEPAD 2 */

            // right bumper: drive back
            if (gp2.right_bumper && !pgp2.right_bumper) {
                Pose currentPose = follower.getPose();
                PathChain goToBucket = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Point(currentPose),
                                        new Point(80, 22),
                                        new Point(scorePose)
                                )
                        )
                        .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                        .build();
                follower.followPath(goToBucket);
            }

            // right trigger: slow mo
            if (gp2.right_trigger > 0.5) {
                speed = 0.5;
            } else {
                speed = 1;
            }

            // dpads: trimming
            if (gp2.dpad_up) {
                extend.manual(1);
            }

            if (gp2.dpad_down) {
                extend.manual(-1);
            }

            if ((!gp2.dpad_up && pgp2.dpad_up) || (!gp2.dpad_down && pgp2.dpad_down)) {
                extend.setTarget(extend.getCurrentPosition());
            }

            if (!follower.isBusy()) {
                // TODO: determine whether to keep these or not
                // idk if these are necessary but they were here earlier
//                follower.breakFollowing();
//                follower.startTeleopDrive();

                follower.setTeleOpMovementVectors(-gp2.left_stick_y * speed, -gp2.left_stick_x * speed, -gp2.right_stick_x * speed, true);
            }
        } else {
            if (gp1.dpad_left && !pgp1.dpad_left) {
                incrementor.decrementIndex();
            } else if (gp1.dpad_right && !pgp1.dpad_right) {
                incrementor.incrementIndex();
            } else if (gp1.dpad_up && !pgp1.dpad_up) {
                incrementor.incrementValue();
            } else if (gp1.dpad_down && !pgp1.dpad_down) {
                incrementor.decrementValue();
            }

            telemetry.addData("x1", incrementor.getValue(0));
            telemetry.addData("x2", incrementor.getValue(1));
            telemetry.addData("x3", incrementor.getValue(2));
            telemetry.addData("x4", incrementor.getValue(3));
            telemetry.addData("editing", incrementor.getCurrentIndex()+1);
        }
    }
}
