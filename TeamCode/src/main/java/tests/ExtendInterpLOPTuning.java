package tests;

import static java.lang.Thread.sleep;
import static shared.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class ExtendInterpLOPTuning extends OpMode {
    private PIDController controller;
    public static double pTop = 0.01, dTop = 0.0005, iTop = 0.01;
    public static double pMiddle = 0.03, dMiddle = 0.001, iMiddle = 0.01;
    public static double pBottom = 0.01, dBottom = 0.0005, iBottom = 0.01;
    public static int target = 0;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;

    private InterpLUT pCoefficients;
    private InterpLUT iCoefficients;
    private InterpLUT dCoefficients;

    private Timer loopTimer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(pBottom, 0, dBottom);
        pCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();
        loopTimer = new Timer();
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

    @Override
    public void loop() {
        double armPos = motorTwo.getCurrentPosition();

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


        // TODO: might want to clamp armPos to be between -20, 780

        double Kp = pCoefficients.get(armPos);
        double Kd = dCoefficients.get(armPos);
        double Ki = iCoefficients.get(armPos);

        controller.setPID(Kp, Ki, Kd);
        double power = controller.calculate(armPos, target);

        try {
            sleep(30);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        motorTwo.setPower(-power);
        motorOne.setPower(-power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("loop time", loopTimer.getElapsedTime());

        loopTimer.resetTimer();
        telemetry.update();
    }
}

