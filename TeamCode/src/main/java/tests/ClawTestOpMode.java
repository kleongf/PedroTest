package tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import shared.Claw;

@TeleOp(name="Claw Test", group="Test")
public class ClawTestOpMode extends OpMode {
    private Claw claw;

    @Override
    public void init() {
        claw = new Claw(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            claw.open();
            telemetry.addData("Claw", "Open");
        } else if (gamepad1.b) {
            claw.close();
            telemetry.addData("Claw", "Closed");
        }

        if (gamepad1.x) {
            claw.spinHorizontal();
            telemetry.addData("Spin", "Horizontal");
        } else if (gamepad1.y) {
            claw.spinVertical();
            telemetry.addData("Spin", "Vertical");
        }

        if (gamepad1.dpad_up) {
            claw.submersibleUp();
            telemetry.addData("Arm", "Submersible Up");
        } else if (gamepad1.dpad_down) {
            claw.submersibleDown();
            telemetry.addData("Arm", "Submersible Down");
        } else if (gamepad1.dpad_left) {
            claw.dropOff();
            telemetry.addData("Arm", "Drop Off");
        } else if (gamepad1.dpad_right) {
            claw.score();
            telemetry.addData("Arm", "Scoring Position");
        }

        telemetry.update();
    }
}

