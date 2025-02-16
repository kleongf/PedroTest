package tests;

import static shared.Constants.*;

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
public class ExtendTuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static int target = 0;
    private DcMotorEx motorOne;
    private DcMotorEx motorTwo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
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
        controller.setPID(p, i, d);
        double armPos = motorTwo.getCurrentPosition();
        double power = controller.calculate(armPos, target);
        motorTwo.setPower(-power);
        motorOne.setPower(-power);
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}

