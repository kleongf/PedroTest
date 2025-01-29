package tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shitfart_Simple")
public class Shitfart_Simple extends com.qualcomm.robotcore.eventloop.opmode.LinearOpMode {

    private DcMotorEx extendMotorOne;
    private DcMotorEx extendMotorTwo;

    @Override
    public void runOpMode() {
        // Hardware mapping as in MainTeleOp
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");

        // Wait for start
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Max power downward control
            if (gamepad1.right_stick_button) {
                extendMotorOne.setPower(-1.0);
                extendMotorTwo.setPower(-1.0);
            } else if (gamepad1.left_stick_button) {
                extendMotorOne.setPower(0.0);
                extendMotorTwo.setPower(0.0);
            }

            // Telemetry
            telemetry.addData("Motor One Power", extendMotorOne.getPower());
            telemetry.addData("Motor Two Power", extendMotorTwo.getPower());
            telemetry.update();
        }
    }
}

