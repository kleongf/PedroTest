package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import shared.Extend;
import shared.Lift;

@TeleOp(name="Intake Testing")
public class HangTesting extends OpMode {
    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    private AnalogInput analogEncoder;

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    private boolean trianglePressed = false;

    Lift lift;
    Extend extend;

    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");
    }

    public void loop() {
        double armLengthInput = gamepad1.left_stick_y;
        if (Math.abs(armLengthInput) > 0.1) {
            extendMotorOne.setPower(armLengthInput * 0.9);
            extendMotorTwo.setPower(armLengthInput * 0.9);
            extend.setTarget(extendMotorTwo.getCurrentPosition());
        } else {
            extend.loop();
        }

        double armAngleInput = gamepad1.right_stick_y;
        if (Math.abs(armAngleInput) > 0.1) {
            liftMotorOne.setPower(armAngleInput * 0.2);
            liftMotorTwo.setPower(armAngleInput * 0.2);
            lift.setTarget(((analogEncoder.getVoltage() / 3.2) * 360 + 145) % 360);
        } else {
            lift.loop();
        }

        if (gamepad1.triangle && !trianglePressed) {
            // if there is not enough power, we will set extend's pid coefficients to something else
            extend.setTarget(0);
        }

        trianglePressed = gamepad1.triangle;

        double y = -this.gamepad2.left_stick_y;
        double x = this.gamepad2.left_stick_x * 1.1;
        double rx = this.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(-frontRightPower);
        backRight.setPower(-backRightPower);
    }
}


