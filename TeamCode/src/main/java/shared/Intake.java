package shared;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import static shared.Constants.*;

public class Intake {
    private final Servo motorOne;
    private final Servo motorTwo;
    private final CRServo intakeMotor;
    private double power = 0;

    public Intake(Servo motorOne, Servo motorTwo, CRServo intakeMotor) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.intakeMotor = intakeMotor;
        motorOne.setPosition(SERVO_UP);
        motorTwo.setPosition(SERVO_UP);
    }

    public void setPower(double p) {
        power = p;
    }

    public void IntakeForward() {
        power = SERVO_FORWARD;
    }

    public void IntakeReverse() {
        power = SERVO_REVERSE;
    }

    public void IntakeStop() {
        power = 0;
    }

    public void loop() {
        intakeMotor.setPower(power);

    }

    public void IntakeUp() {
        motorOne.setPosition(SERVO_UP);
        motorTwo.setPosition(SERVO_UP);
    }

    public void IntakeDown() {
        // was 0.22
        motorOne.setPosition(SERVO_DOWN);
        motorTwo.setPosition(SERVO_DOWN);
    }

}

