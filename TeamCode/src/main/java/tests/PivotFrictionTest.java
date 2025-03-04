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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.*;

import shared.Multiplier;
import tests.LinearRegression;

// logic:
/*
if error > some threshold:
    if its moving: (velocity is not really low)
        apply kv in the sign of the direction needed to travel
    if its not moving: (velocity is really low)
        apply ks in the sign of the direction needed to travel
*/

@Config
@TeleOp
public class PivotFrictionTest extends OpMode {
    public static double powerIncrement = 0.0003;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;
    private DcMotorEx extendMotor;
    private AnalogInput encoder;
    private double direction = -1;
    private double power = 0.0;
    private double prevPos;
    private ArrayList<double[]> points;
    public Timer timer;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        points = new ArrayList<>();
    }

    @Override
    public void loop() {

        // the hold in place power should probably be the start power
        double armPos = ((encoder.getVoltage() / 3.231 * 360)) % 360;
        double dx = armPos - prevPos;
        double dt = timer.getElapsedTimeSeconds();
        double velocity = dx/dt;

        if (armPos < 140 && power < 0.9) {
            power += powerIncrement;
            double[] temp = {power, velocity};
            points.add(temp);

            motorOne.setPower(direction * power);
            motorTwo.setPower(direction * power);

            prevPos = armPos;
            timer.resetTimer();

            telemetry.addData("velocity", velocity);
            telemetry.addData("power", power);
        } else {
            double[] line = LinearRegression.computeBestFitLine(points);
            telemetry.addData("kS", line[0]);
            telemetry.addData("kV", line[1]);
        }
        telemetry.update();
    }
    @Override
    public void start() {
        timer.resetTimer();
    }

}