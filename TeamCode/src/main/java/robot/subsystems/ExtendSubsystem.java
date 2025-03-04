package robot.subsystems;

import static robot.RobotConstants.*;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExtendSubsystem {
    private final PIDController controller;
    public static double p = 0.01, i = 0.0, d = 0.0;
    public static int target = 0;
    private final DcMotorEx motorOne;
    private final DcMotorEx motorTwo;
    private boolean manual = false;
    private double pwr = -0.6;

    public ExtendSubsystem(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);
        motorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
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

//    private double clamp(double x) {
//        if (x >= 780) {
//            return 770;
//        } else if (x <= -20) {
//            return -20;
//        }
//        return x;
//    }

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
        motorTwo.setPower(pwr * d);
        motorTwo.setPower(pwr * d);
    }

    public int getCurrentPosition() {
        return motorTwo.getCurrentPosition();
    }
}
