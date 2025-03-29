package robot;

import static com.pedropathing.util.Constants.setConstants;

import static robot.SpecRobotConstants.*;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robot.subsystems.ExtendSubsystem;
import robot.subsystems.IntakeSubsystem;
import robot.subsystems.LiftSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class SpecRobot {
    List<LynxModule> allHubs;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ExtendSubsystem extend;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    public Timer grabTimer, prepareGrabTimer, depositTimer, prepareTimer;
    public int grabState, depositState, prepareState, prepareGrabState;

    public SpecRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        extend = new ExtendSubsystem(this.hardwareMap);
        lift = new LiftSubsystem(this.hardwareMap);
        intake = new IntakeSubsystem(this.hardwareMap);

        setConstants(FConstants.class, LConstants.class);

        allHubs = this.hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        grabTimer = new Timer();
        depositTimer = new Timer();
        prepareTimer = new Timer();
        prepareGrabTimer = new Timer();
    }

    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        grab();
        deposit();
        prepareScore();
        prepareGrab();

        extend.loop();
        lift.loop();
        telemetry.update();
    }

    public void start() {
    }

    public void prepareGrab() {
        switch (prepareGrabState) {
            case 1:
                intake.setTarget(INTAKE_GRAB);
                extend.setTarget(EXTEND_GRAB);
                setPrepareGrabState(2);
                break;
            case 2:
                if (prepareGrabTimer.getElapsedTimeSeconds() > 0.3) {
                    lift.setTarget(LIFT_GRAB);
                    setPrepareGrabState(-1);
                }
                break;
        }
    }

    public void setPrepareGrabState(int x) {
        prepareGrabState = x;
        prepareGrabTimer.resetTimer();
    }

    public void startPrepareGrab() {
        setPrepareGrabState(1);
    }

    public boolean isPrepareGrabBusy() {
        return prepareGrabState != -1;
    }


    public void grab() {
        switch (grabState) {
            case 1:
                intake.close();
                setGrabState(2);
                break;
            case 2:
                if (grabTimer.getElapsedTimeSeconds() > 0.2) {
                    setGrabState(-1);
                }
                break;
        }
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
                extend.setTarget(EXTEND_SCORE);
                setDepositState(2);
                break;
            case 2:
                if (depositTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.open();
                    setDepositState(3);
                }
                break;
            case 3:
                if (depositTimer.getElapsedTimeSeconds() > 0.1) {
                    setDepositState(-1);
                }
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
                lift.setTarget(LIFT_SCORE);
                intake.setTarget(INTAKE_SCORE);
                setPrepareState(2);
                break;
            case 2:
                if (prepareTimer.getElapsedTimeSeconds() > 0.3) {
                    extend.setTarget(EXTEND_PREPARE);
                    setPrepareState(3);
                }
                break;
            case 3:
                if (prepareTimer.getElapsedTimeSeconds() > 0.2) {
                    setPrepareState(-1);
                }
                break;
        }
    }

    public void setPrepareState(int x) {
        prepareState = x;
        prepareTimer.resetTimer();
    }

    public void startPrepareScore() {
        setPrepareState(1);
    }

    public boolean isPrepareScoreBusy() {
        return prepareState != -1;
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

}

