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

    private static final double TICKS_PER_INCH = 29;

    private static final double length_1_ticks = 10.0 * TICKS_PER_INCH;
    private static final double length_2_ticks = 8.0 * TICKS_PER_INCH;
    private static final double length_3_ticks = 7.0 * TICKS_PER_INCH;
    private static final double length_4_ticks = 5.0 * TICKS_PER_INCH;
    private static final double length_last_ticks = 5.0 * TICKS_PER_INCH;

    // Segment masses as fractions of the total mass
    private static final double m_1 = 4.0 / 11.0;
    private static final double m_2 = 3.0 / 11.0;
    private static final double m_3 = 2.0 / 11.0;
    private static final double m_4 = 1.5 / 11.0;
    private static final double m_end = 1.5 / 11.0;

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
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        //double ffcoef = 1 + (extendMotor.getCurrentPosition() / 700.0) * 1.7;
        double scale = extendMotor.getCurrentPosition() / length_1_ticks;  // Scale based on the maximum length (in ticks)

        // Compute the adjusted distances based on the current arm extension
        double d1 = (length_1_ticks / 2.0) * scale;
        double d2 = (length_2_ticks / 2.0) * scale;
        double d3 = (length_3_ticks / 2.0) * scale;
        double d4 = (length_4_ticks / 2.0) * scale;
        double d_end = length_last_ticks * scale;

        // Compute the torque for each segment
        double torque_1 = m_1 * d1;
        double torque_2 = m_2 * d2;
        double torque_3 = m_3 * d3;
        double torque_4 = m_4 * d4;
        double torque_end = m_end * d_end; // Point mass at the end

        // Sum all torques to get total torque
        // double ffcoef = (torque_1 + torque_2 + torque_3 + torque_4 + torque_end);

        // TODO: if this code doesn't work uncomment this
        // double ffcoef = 1 + (extendMotor.getCurrentPosition() / 700) * 2;
        double ffcoef = 0.3 + (extendMotor.getCurrentPosition() / 700.0) * 2.5;
        double armPos = ((encoder.getVoltage() / 3.235 * 360) + offset + inherentOffset) % 360;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians((encoder.getVoltage() / 3.235 * 360))) * f * ffcoef;
        double power = pid + ff;
        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("extend", extendMotor.getCurrentPosition());
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
}



