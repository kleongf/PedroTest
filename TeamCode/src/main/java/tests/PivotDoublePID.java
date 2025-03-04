package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import shared.Multiplier;

@Config
@TeleOp
public class PivotDoublePID extends OpMode {
    private PIDFController controller;
    public static double pTop = 0.02, dTop = 0.001;
    public static double pMiddle = 0.04, dMiddle = 0.002;
    public static double pBottom = 0.02, dBottom = 0.001;

    public static double f = 0.002;
    public static double target = 59;
    private static double inherentOffset = 59;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private DcMotorEx extendMotor;
    private AnalogInput encoder;
    private Multiplier multiplier;

    private InterpLUT pCoefficients;
    private InterpLUT dCoefficients;

    public static boolean secondaryPIDEnabled = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDFController(pBottom, 0, dBottom, 0);
        motorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        motorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        extendMotor = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        encoder = hardwareMap.get(AnalogInput.class, "encoder");

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);
        multiplier = new Multiplier();


        pCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();
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
        double fgcoef = multiplier.calculateMultiplier(extendMotor.getCurrentPosition());
        double fg = Math.cos(Math.toRadians((encoder.getVoltage() / 3.231 * 360) - inherentOffset)) * f * fgcoef;

        // different p and d values for different parts
        // from top to middle: low p and low d
        // middle: high p and d
        // bottom: low p and low d
        // very bottom: biggest p and biggest d
        // this will get us closer to the trapezoidal motion profile

        pCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();

        pCoefficients.add(55, pBottom);
        pCoefficients.add(110, pMiddle);
        pCoefficients.add(170, pTop);

        dCoefficients.add(55, dBottom);
        dCoefficients.add(110, dMiddle);
        dCoefficients.add(170, dTop);

        pCoefficients.createLUT();
        dCoefficients.createLUT();

        double Kp = pCoefficients.get(armPos);
        double Kd = dCoefficients.get(armPos);

        controller.setPIDF(Kp, 0, Kd, 0);

        double pid = controller.calculate(armPos, target);

        double power = pid + fg;

        motorOne.setPower(-power);
        motorTwo.setPower(-power);

        telemetry.addData("pos", armPos);
        telemetry.addData("power", -power);
        telemetry.addData("target", target);
        telemetry.addData("extend", extendMotor.getCurrentPosition());
        telemetry.addData("voltage", encoder.getVoltage());
        telemetry.update();

    }
}
