package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import shared.Multiplier;

@Config
@TeleOp
public class PDFLTuning extends OpMode {
    private PIDFController controller;
    public static double p = 0.0, i = 0, d = 0.00;
    public static double frictionCoefficient = 1.0;
    public static double f = 0;
    public static double target = 59;
    private static double inherentOffset = 59;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private DcMotorEx extendMotor;
    private AnalogInput encoder;
    private Multiplier multiplier;
    public static double f1 = 0.0;
    public static double kV = 0.0, kS = 0.0;
    private double prevPos;
    public Timer timer;

    // concern: the friction overcoming thing might be too high and causes some jerk
    // this leads to oscillation
    // solution: if the error threshold is not made, make sure that the power does not exceed the threshold
    // im afraid this will lead to over/undershooting with larger movements though
    // so solution: also check to see what the motor velocities are

    public static double errorThreshold = 1.0;

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
        timer = new Timer();
        prevPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
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
        controller.setPIDF(p, i , d, 0);
        // velocity = dx/dt measured in degrees/second
        double dt = timer.getElapsedTimeSeconds();
        double armPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
        double dx = armPos - prevPos;
        double velocity = dx/dt;

        double fgcoef = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double pid = controller.calculate(armPos, target);
        double fg = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360) - inherentOffset)) * f * fgcoef;
        double power = 0;

        // TODO:
        // there is no velocity because the motor is not connected to an encoder
        // rn this is degrees/second

        if (Math.abs(target-armPos) > errorThreshold && Math.abs(velocity) < 20) {
            double ff = f1 * sgn(target-armPos);

            if (Math.abs(pid + fg) < Math.abs(ff)) {
                // make sure that there is enough power to move the arm
                power = ff;
            } else {
                // idk if this will work lol
                if (sgn(target-armPos) > 0) {
                    power = pid + ff - fg;
                } else {
                    power = pid + ff + fg;
                }
            }
        } else {
            // motor is moving fast (and has a lot of impulse) OR is within target range OR has sufficient power
            power = pid + fg;
        }
        prevPos = armPos;
        timer.resetTimer();

        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("velocity", velocity);
        telemetry.addData("power", -power);
        telemetry.addData("target", target);
        telemetry.addData("extend", extendMotor.getCurrentPosition());
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
    @Override
    public void start() {
        timer.resetTimer();
    }
}
