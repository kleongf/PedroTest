package shared;

import static shared.Constants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private final PIDController controller;
    public static double p = 0.02, i = 0.01, d = 0;
    public static double f = 0.1;
    public static double target = ANGLE_ZERO;
    // keep testing offset
    private static double offset = ANGLE_ZERO;
    private static double inherentOffset = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final AnalogInput encoder;

    public Lift(DcMotorEx motorOne, DcMotorEx motorTwo, AnalogInput encoder) {
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.encoder = encoder;
    }

    public void setTarget(double t) {
        if (t >= ANGLE_STOP_MAX) {
            target = ANGLE_STOP_MAX;
        } else target = Math.max(t, ANGLE_STOP_MIN);
    }

    public double getTarget() { return target; }

    public void loop() {
        double armPos = ((encoder.getVoltage() / 3.235 * 360) + offset + inherentOffset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.235 * 360))) * f;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);
    }
}


