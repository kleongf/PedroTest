package shared;

import static shared.Constants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Extend {
    private PIDController controller;
    // 0.025, 0.02, 0.0005
    public static double p = 0.025, i = 0.02, d = 0.0005;
//    public static double p = 0.03, i = 0.0, d = 0.0001;
    public static double f = 0;
    public static int target = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private boolean manual = false;

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
        manual = false;
        if (t >= EXTEND_STOP_MAX) {
            target = EXTEND_STOP_MAX;
        } else target = Math.max(t, EXTEND_STOP_MIN);
    }

    public int getTarget() { return target; }

    public void loop() {
        if (!manual) {
            double armPos = motorTwo.getCurrentPosition();
            double power = controller.calculate(armPos, target);
            motorTwo.setPower(-power);
            motorOne.setPower(-power);
        }
    }

    // -1 for back, 1 for forward
    public void manual(int d) {
        manual = true;
        motorTwo.setPower(-0.6 * d);
        motorTwo.setPower(-0.6 * d);
    }

    public int getCurrentPosition() {
        return motorTwo.getCurrentPosition();
    }
}
