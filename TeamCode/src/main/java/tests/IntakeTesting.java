package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import shared.Intake;

@TeleOp(name="Intake Testing")
public class IntakeTesting extends OpMode {
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;

    Intake intake;

    // using dpad controls to test
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    public void init() {
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);
    }

    public void loop() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            intake.IntakeUp();
            // seems to be moving it down. also make the angle less down by 0.1
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            intake.IntakeDown();
            // moves it up, but not high enough
        }

        if (gamepad1.dpad_left && !dpadLeftPressed) {
            intake.IntakeReverse(); // outtake
        }

        if (gamepad1.dpad_right && !dpadRightPressed) {
            intake.IntakeForward();
        }
        dpadRightPressed = gamepad1.dpad_right;
        dpadLeftPressed = gamepad1.dpad_left;
        dpadUpPressed = gamepad1.dpad_up;
        dpadDownPressed = gamepad1.dpad_down;

        intake.loop();
    }
}

