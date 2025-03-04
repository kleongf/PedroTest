package shared;

import static shared.Constants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Extend {
    private PIDController controller;
    // public static double p = 0.03, i = 0.0, d = 0.0003;

    public static double pTop = 0.012, dTop = 0.000, iTop = 0.0;
    public static double pMiddle = 0.012, dMiddle = 0.00, iMiddle = 0.0;
    public static double pBottom = 0.012, dBottom = 0.000, iBottom = 0.0;

    private static double errorThreshold = 5;

    private InterpLUT pCoefficients;
    private InterpLUT dCoefficients;
    private InterpLUT iCoefficients;

    public static int target = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private boolean manual = false;
    private double pwr = -0.6;

    public Extend(DcMotorEx motorOne, DcMotorEx motorTwo) {
        pCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();
        iCoefficients = new InterpLUT();

        pCoefficients.add(-30, pBottom);
        pCoefficients.add(350, pMiddle);
        pCoefficients.add(780, pTop);

        dCoefficients.add(-30, dBottom);
        dCoefficients.add(350, dMiddle);
        dCoefficients.add(780, dTop);

        iCoefficients.add(-30, iBottom);
        iCoefficients.add(350, iMiddle);
        iCoefficients.add(780, iTop);

        pCoefficients.createLUT();
        dCoefficients.createLUT();
        iCoefficients.createLUT();

        controller = new PIDController(pBottom, 0, dBottom);
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

    public void setManualPower(double x) {
        pwr = -x;
    }

    public void setTarget(int t) {
        manual = false;
        if (t >= EXTEND_STOP_MAX) {
            target = EXTEND_STOP_MAX;
        } else target = Math.max(t, EXTEND_STOP_MIN);
    }

    public int getTarget() { return target; }

    private double clamp(double x) {
        if (x >= 780) {
            return 770;
        } else if (x <= -20) {
            return -20;
        }
        return x;
    }

    public void loop() {
        if (!manual) {
            double armPos = motorTwo.getCurrentPosition();
            double Kp = pCoefficients.get(clamp(armPos));
            double Kd = dCoefficients.get(clamp(armPos));
            double Ki = iCoefficients.get(clamp(armPos));

            controller.setPID(Kp, Ki, Kd);

            double power = controller.calculate(armPos, target);


            motorTwo.setPower(-power);
            motorOne.setPower(-power);

        }
    }

    // -1 for back, 1 for forward
    public void manual(int d) {
        manual = true;
        motorTwo.setPower(pwr * d);
        motorTwo.setPower(pwr * d);
    }

    public int getCurrentPosition() {
        return motorTwo.getCurrentPosition();
    }
}
