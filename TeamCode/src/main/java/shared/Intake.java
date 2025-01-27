package shared;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final Servo motorOne;
    private final Servo motorTwo;
    private final CRServo intakeMotor;
    private double power = 0;

    public Intake(Servo motorOne, Servo motorTwo, CRServo intakeMotor) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.intakeMotor = intakeMotor;
        motorOne.setPosition(0);
        motorTwo.setPosition(0);
    }

    public void setPower(double p) {
        power = p;
    }

    public void IntakeForward() {
        power = -0.5;
    }

    public void IntakeReverse() {
        power = 0.5;
    }

    public void IntakeStop() {
        power = 0;
    }

    public void loop() {
        intakeMotor.setPower(power);
    }

    public void IntakeUp() {
        motorOne.setPosition(0);
        motorTwo.setPosition(0);
    }

    public void IntakeDown() {
        motorOne.setPosition(0.7);
        motorTwo.setPosition(0.7);
    }

    public void IntakeDefault() {
        motorOne.setPosition(0.4);
        motorOne.setPosition(0.4);
    }
}

