package shared;

import com.qualcomm.robotcore.hardware.Servo;
import static shared.Constants.*;

public class Claw {
    private final Servo angleMotorOne;
    private final Servo angleMotorTwo;
    private final Servo intakeMotor;
    private final Servo spinMotor;

    public Claw(Servo angleMotorOne, Servo angleMotorTwo, Servo intakeMotor, Servo spinMotor) {
        this.angleMotorOne = angleMotorOne;
        this.angleMotorTwo = angleMotorTwo;
        this.intakeMotor = intakeMotor;
        this.spinMotor = spinMotor;
        angleMotorOne.setPosition(SERVO_UP);
        angleMotorTwo.setPosition(SERVO_UP);
        intakeMotor.setPosition(0);
        spinMotor.setPosition(0);
    }

    public void tighten() {
        intakeMotor.setPosition(1);
    }
    public void release() {
        intakeMotor.setPosition(0);
    }

    public void spinHorizontal() {
        // move to 90 degrees = 0.5
        spinMotor.setPosition(0.5);
    }

    public void spinVertical() {
        spinMotor.setPosition(0);
    }

    public void loop() {}

    public void clawUp() {
        angleMotorOne.setPosition(SERVO_UP);
        angleMotorTwo.setPosition(SERVO_UP);
    }

    public void clawDown() {
        angleMotorOne.setPosition(0.24);
        angleMotorTwo.setPosition(0.24);
    }
}
