package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import shared.Multiplier;

@Config
@TeleOp
public class PivotTuning extends OpMode {
    private PIDController controller;
    public static double p = 0.0, i = 0, d = 0.00;
    public static double f = 0;
    public static double target = 10;
    private static double offset = 10;
    private static double inherentOffset = 0;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private DcMotorEx extendMotor;
    private AnalogInput encoder;
    private Multiplier multiplier;

    // best values so far: d=0.002, p=0.08, i=0.02, f=0.0022
    // however it seems to be doing fine so...

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
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

    @Override
    public void loop() {
        controller.setPID(p, i, d);

        double ffcoef = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double armPos = ((encoder.getVoltage() / 3.231 * 360) + offset + inherentOffset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360))) * f * ffcoef;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("power", power);
        telemetry.addData("target", target);
        telemetry.addData("extend", extendMotor.getCurrentPosition());
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
}



