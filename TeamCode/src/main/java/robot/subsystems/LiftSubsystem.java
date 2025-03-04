package robot.subsystems;

import static robot.RobotConstants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import shared.Multiplier;

public class LiftSubsystem {
    private final PIDController controller;
    public static double p = 0.03, i = 0.0, d = 0.0012;
    public static double frictionCoefficient = 1.3;
    public static double f1 = 0.2;
    public static double f = 0.002;
    public static double target = 59;
    private final double inherentOffset = 59;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final AnalogInput encoder;
    private final DcMotorEx extendMotor;
    private final Multiplier multiplier;

    private double prevPos;
    public Timer timer;
    public static double errorThreshold = 0.5;

    public LiftSubsystem(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        motorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        this.multiplier = new Multiplier();
        this.timer = new Timer();
        timer.resetTimer();
        this.prevPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public int sgn(double x) {
        if (x > 0) {
            return 1;
        } else if (x < 0) {
            return -1;
        }
        return 0;
    }

    public void setTarget(double t) {
        if (t >= ANGLE_STOP_MAX) {
            target = ANGLE_STOP_MAX;
        } else target = Math.max(t, ANGLE_STOP_MIN);
    }

    public double getTarget() { return target; }

    public double getPower() {
        double coefficient = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double armPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360))) * f * coefficient;
        return pid + ff;
    }

    public void loop() {
        double dt = timer.getElapsedTimeSeconds();
        double armPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
        double dx = armPos - prevPos;
        double velocity = dx/dt;
        double fgcoef = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double pid = controller.calculate(armPos, target);
        double fg = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360) - inherentOffset)) * f * fgcoef;
        double power = 0;

        if (Math.abs(target-armPos) > errorThreshold && Math.abs(velocity) < 20) {
            double ff = f1 * sgn(target-armPos);
            if (sgn(target-armPos) > 0) {
                ff *= frictionCoefficient;
            }
            // if we need to move up (sgn > 0) then f1 should be multiplied by another factor to counteract fg
            if (Math.abs(pid + fg) < Math.abs(ff)) {
                // make sure that there is enough power to move the arm
                power = ff;
            } else {
                power = pid + fg;
            }
        } else {
            // motor is moving fast (and has a lot of impulse) OR is within target range OR has sufficient power
            power = pid + fg;
        }
        prevPos = armPos;
        timer.resetTimer();
        motorOne.setPower(-power);
        motorTwo.setPower(-power);
    }
}

