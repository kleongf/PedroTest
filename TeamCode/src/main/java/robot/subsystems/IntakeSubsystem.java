package robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    private final Servo angleMotorOne;
    private final Servo angleMotorTwo;
    private final Servo intakeMotorOne;
    private final Servo intakeMotorTwo;
    private final Servo spinMotor;
    private boolean horizontal = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        this.angleMotorOne = hardwareMap.get(Servo.class, "angleMotorOne");
        this.angleMotorTwo = hardwareMap.get(Servo.class, "angleMotorTwo");
        this.angleMotorOne.setDirection(Servo.Direction.REVERSE);
        this.intakeMotorOne = hardwareMap.get(Servo.class, "intakeMotorOne");
        this.intakeMotorTwo = hardwareMap.get(Servo.class, "intakeMotorTwo");
        this.spinMotor = hardwareMap.get(Servo.class, "spinMotor");

        close();
        spinVertical();
    }

    public void toggleSpin() {
        if (horizontal) {
            spinVertical();
        } else {
            spinHorizontal();
        }
    }

    public void close() {
        intakeMotorOne.setPosition(0.13);
        intakeMotorTwo.setPosition(0.84);
    }
    public void open() {
        intakeMotorOne.setPosition(0.57);
        intakeMotorTwo.setPosition(0.4);
    }

    public void spinHorizontal() {
        // move to 90 degrees = 0.5
        spinMotor.setPosition(0.78);
        horizontal = true;
    }

    public void spinVertical() {
        spinMotor.setPosition(0.32);
        horizontal = false;
    }


    public void down() {
        angleMotorOne.setPosition(0);
        angleMotorTwo.setPosition(0);
    }

    public void up() {
        angleMotorOne.setPosition(1);
        angleMotorTwo.setPosition(1);
    }

    public void mid() {
        angleMotorOne.setPosition(0.5);
        angleMotorTwo.setPosition(0.5);
    }
}

