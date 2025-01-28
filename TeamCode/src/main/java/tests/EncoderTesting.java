package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class EncoderTesting extends OpMode {
    private PIDController controller;
    public static double p = 0.0, i = 0, d = 0.00;
    public static double f = 0;
    public static double target = 90;
    private static double offset = 90;
    private static double inherentOffset = 0;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private AnalogInput encoder;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        motorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double armPos = ((encoder.getVoltage() / 3.235 * 360) + offset + inherentOffset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.235 * 360))) * f;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
}


