package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import shared.Claw;

@Config
@TeleOp
public class ClawMotorTest extends OpMode {
    private Claw claw;

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevDpad = false;

    private boolean im1 = false;
    private boolean im2 = false;
    private boolean am1 = false;
    private boolean am2 = false;
    private boolean sm1 = false;

    public static double im1open = 1;
    public static double im1closed = 0;

    public static double im2open = 0;
    public static double im2closed = 1;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a && !prevA) {
            if (im1) {
                claw.im1(im1open);
                telemetry.addData("IntakeMotorOne", im1open);
            } else {
                claw.im1(im1closed);
                telemetry.addData("IntakeMotorOne", im1closed);
            }
            im1 = !im1;
        } else if (gamepad1.b && !prevB) {
            if (im2) {
                claw.im2(im2open);
                telemetry.addData("IntakeMotorTwo", im2open);
            } else {
                claw.im2(im2closed);
                telemetry.addData("IntakeMotorTwo", im2closed);
            }
            im2 = !im2;
        }

        if (gamepad1.x && !prevX) {
            if (am1) {
                claw.am1(1);
                telemetry.addData("AngleMotorOne", "1");
            } else {
                claw.am1(0);
                telemetry.addData("AngleMotorOne", "0");
            }
            am1 = !am1;
        } else if (gamepad1.y && !prevY) {
            if (am2) {
                claw.am2(1);
                telemetry.addData("AngleMotorTwo", "1");
            } else {
                claw.am2(0);
                telemetry.addData("AngleMotorTwo", "0");
            }
            am2 = !am2;
        }

        if (gamepad1.dpad_up && !prevDpad) {
            if (sm1) {
                claw.sm1(0.32);
                telemetry.addData("Spinmotor", "vert");
            } else {
                claw.sm1(0.78);
                telemetry.addData("Spinmotor", "horiz");
            }
            sm1 = !sm1;
        }

        prevA = gamepad1.a;
        prevB = gamepad1.b;
        prevX = gamepad1.x;
        prevY = gamepad1.y;
        prevDpad = gamepad1.dpad_up;

        telemetry.update();
    }
}
