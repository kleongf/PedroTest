package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import shared.Multiplier;

@Config
@TeleOp
public class PivotTuning extends OpMode {
    private PIDFController controller;
    public static double p = 0.0, i = 0, d = 0.00;
    public static double f = 0;
    public static double target = 59;
    private static double inherentOffset = 59;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private DcMotorEx extendMotor;
    private AnalogInput encoder;
    private Multiplier multiplier;

    // public static double p1 = 0.0, i1 = 0.0, d1 = 0.0,
    public static double f1 = 0.0;
    public static boolean secondaryPIDEnabled = true;

    // best values so far: d=0.002, p=0.08, i=0.02, f=0.0022
    // however it seems to be doing fine so...

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(p, i, d, 0);
        controller.setPIDF(p, i, d, 0);
        motorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);
        multiplier = new Multiplier();
    }

    public int sgn(double x) {
        if (x > 0) {
            return 1;
        } else if (x < 0) {
            return -1;
        }
        return 0;
    }

    @Override
    public void loop() {
        double armPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;

        // just tune a feedforward constant
//        if (Math.abs(armPos-target) < 10 && Math.abs(armPos-target) > 0.5) {
//            controller.setPIDF(p1, i1, d1, 0);
//        } else {
//            controller.setPIDF(p, i, d, 0);
//            // Kf: a constant needed to overcome the friction force
//        }
        controller.setPIDF(p, i , d, 0);

        double fgcoef = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double pid = controller.calculate(armPos, target);
        double fg = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360) - inherentOffset)) * f * fgcoef;
        double power = 0;

        // small movements: add a feed forward to counteract friction
        // && Math.abs(motorOne.getVelocity() < something) this could prevent it from moving too fast (ticks/s)
        // at the bottom of large movements but tbh i dont think it will matter
        if (target < 70 && Math.abs(armPos-target) < 10 && Math.abs(armPos-target) > 0.5 && Math.abs(motorOne.getVelocity()) < 200) {
            // ff neg: arm goes up
            // ff pos: arm goes down
            // if target < armpos, then ff is pos
            double ff = f1 * sgn(target-armPos);
            power = pid + fg + ff;
        } else {
            power = pid + fg;
        }

        // TODO: I JUST REALIZED THAT FG MAY NEED TO BE NEGATIVE?
        // CHANGE THIS IF IT WORKS

        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("velocity", motorOne.getVelocity());
        telemetry.addData("power", -power);
        telemetry.addData("target", target);
        telemetry.addData("extend", extendMotor.getCurrentPosition());
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
}



