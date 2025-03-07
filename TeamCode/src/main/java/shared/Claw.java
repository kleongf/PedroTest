package shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static shared.Constants.*;

public class Claw {

    private final Servo angleMotorOne;
    private final Servo angleMotorTwo;
    private final Servo intakeMotorOne;
    private final Servo intakeMotorTwo;
    private final Servo spinMotor;
    private boolean horizontal = false;

    public Claw(HardwareMap hardwareMap) {
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

    public void am1(double x) {
        angleMotorOne.setPosition(x);
    }

    public void am2(double x) {
        angleMotorTwo.setPosition(x);
    }

    public void im1(double x) {
        intakeMotorOne.setPosition(x);
    }

    public void im2(double x) {
        intakeMotorTwo.setPosition(x);
    }

    public void sm1(double x) {
        spinMotor.setPosition(x);
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

    public void score() {
        angleMotorOne.setPosition(1);
        angleMotorTwo.setPosition(1);
    }

    public void submersibleDown() {
        angleMotorOne.setPosition(0);
        angleMotorTwo.setPosition(0);
    }

    public void submersibleUp() {
        angleMotorOne.setPosition(1);
        angleMotorTwo.setPosition(1);
    }

    public void submersibleMid() {
        angleMotorOne.setPosition(0.5);
        angleMotorTwo.setPosition(0.5);
    }

    public void dropOff() {
        angleMotorOne.setPosition(0);
        angleMotorTwo.setPosition(0);
    }

    public class OpenClawRunnable implements Runnable {
        @Override
        public void run() {
            open();
        }
    }
    public Runnable OpenClawRunnable() { return new OpenClawRunnable();}



}
