package shared;

import static shared.Constants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private final PIDController controller;
    public static double p = 0.08, i = 0.02, d = 0.002;
    public static double f = 0.0022;
    public static double target = ANGLE_ZERO;
    // keep testing offset
    private static double offset = ANGLE_ZERO;
    private static double inherentOffset = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final AnalogInput encoder;
    private final DcMotorEx extendMotor;
    private final Multiplier multiplier;

    public Lift(DcMotorEx motorOne, DcMotorEx motorTwo, AnalogInput encoder, DcMotorEx extendMotor) {
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.encoder = encoder;
        this.extendMotor = extendMotor;
        this.multiplier = new Multiplier();
        // maybe this is causing the problem? idk
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setTarget(double t) {
        if (t >= ANGLE_STOP_MAX) {
            target = ANGLE_STOP_MAX;
        } else target = Math.max(t, ANGLE_STOP_MIN);
    }

    public double getTarget() { return target; }

    public void loop() {
        // was 1 and 1.5
        double coefficient = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        // double multiplier = 1 + (extendMotor.getCurrentPosition() / 700.0) * 1.5;
        double armPos = ((encoder.getVoltage() / 3.235 * 360) + offset + inherentOffset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.235 * 360))) * f * coefficient;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);
    }
}


