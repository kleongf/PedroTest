package shared;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;

public class Extend {
    private PIDController controller;
    public static double p = 0.025, i = 0.02, d = 0.0005;
    public static double f = 0;
    public static int target = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;

    // for future reference: maybe put all constants together in a separate class?
    private static int HIGH = 700;
    private static int LOW = 300;
    private static int ZERO = 0;
    private static int DEFAULT = 100;

    public Extend(DcMotorEx motorOne, DcMotorEx motorTwo) {
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOne.setDirection(DcMotor.Direction.REVERSE);
        motorTwo.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setTarget(int t) {
        if (t >= 720) {
            target = 720;
        } else target = Math.max(t, 0);
    }

    public int getTarget() { return target; }

    public void loop() {
        double armPos = motorTwo.getCurrentPosition();
        double power = controller.calculate(armPos, target);
        motorTwo.setPower(-power);
        motorOne.setPower(-power);
    }
}
