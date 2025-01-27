package shared;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Lift {
    private final PIDController controller;
    public static double p = 0.05, i = 0.02, d = 0.001;
    public static double f = 0;
    public static double target = 150;
    // keep testing offset
    private static double offset = 150;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private final AnalogInput encoder;

    // for future reference: maybe put all constants together in a separate class?
    private static int UP = 240;
    private static int DOWN = 145;
    // set default to offset
    private static int DEFAULT = 150;

    public Lift(DcMotorEx motorOne, DcMotorEx motorTwo, AnalogInput encoder) {
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.encoder = encoder;
    }

    public void setTarget(double t) {
        if (t >= 250) {
            target = 250;
        } else target = Math.max(t, 143);
    }

    public double getTarget() { return target; }

    public void loop() {
        double armPos = ((encoder.getVoltage() / 3.2 * 360) + offset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(armPos)) * f;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);
    }
}


