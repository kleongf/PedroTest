package shared;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private final Servo motorOne;
    private final Servo motorTwo;
    private final CRServo intakeMotor;
    private double power = 0;

    public Intake(Servo motorOne, Servo motorTwo, CRServo intakeMotor) {
        this.motorOne = motorOne;
        this.motorTwo = motorTwo;
        this.intakeMotor = intakeMotor;
        motorOne.setPosition(0.8);
        motorTwo.setPosition(0.8);
    }

    public void setPower(double p) {
        power = p;
    }

    public void IntakeForward() {
        power = -0.3;
    }

    public void IntakeReverse() {
        power = 0.3;
    }

    public void IntakeStop() {
        power = 0;
    }

    public void loop() {
        intakeMotor.setPower(power);

    }

    public void IntakeUp() {
        motorOne.setPosition(0.8);
        motorTwo.setPosition(0.8);
    }

    public void IntakeDown() {
        // was 0.22
        motorOne.setPosition(0.22);
        motorTwo.setPosition(0.22);
    }

}

