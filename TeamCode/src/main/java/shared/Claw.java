package shared;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static shared.Constants.*;

public class Claw {
//    public enum GrabState {
//        CLOSED, OPEN
//    }
//
//    public enum PivotState {
//        SCORE, SUBMERSIBLE_DOWN, DROP_OFF, SUBMERSIBLE_UP
//    }
//
//    public enum RotateState {
//        VERTICAL, HORIZONTAL
//    }

    private final Servo angleMotorOne;
    private final Servo angleMotorTwo;
    private final Servo intakeMotor;
    private final Servo spinMotor;

//    public GrabState grabState;
//    public RotateState rotateState;
//    public PivotState pivotState;


    public Claw(HardwareMap hardwareMap) {
        this.angleMotorOne = hardwareMap.get(Servo.class, "angleMotorOne");
        this.angleMotorTwo = hardwareMap.get(Servo.class, "angleMotorTwo");
        this.intakeMotor = hardwareMap.get(Servo.class, "intakeMotor");
        this.spinMotor = hardwareMap.get(Servo.class, "spinMotor");

        close();
        submersibleUp();
        spinVertical();
    }


    public void close() {
        intakeMotor.setPosition(1);
    }
    public void open() {
        intakeMotor.setPosition(0);
    }

    public void spinHorizontal() {
        // move to 90 degrees = 0.5
        spinMotor.setPosition(0.5);
    }

    public void spinVertical() {
        spinMotor.setPosition(0);
    }

    public void score() {
        angleMotorOne.setPosition(0.8);
        angleMotorTwo.setPosition(0.8);
    }

    public void submersibleDown() {
        angleMotorOne.setPosition(0.24);
        angleMotorTwo.setPosition(0.24);
    }

    public void submersibleUp() {
        angleMotorOne.setPosition(0.5);
        angleMotorTwo.setPosition(0.5);
    }

    public void dropOff() {
        angleMotorOne.setPosition(0);
        angleMotorTwo.setPosition(0);
    }

//    public void setGrabState(GrabState grabState) {
//        switch (grabState) {
//            case CLOSED:
//                close();
//                break;
//            case OPEN:
//                open();
//                break;
//        }
//    }
//
//    public void setRotateState(RotateState rotateState) {
//        switch (rotateState) {
//            case HORIZONTAL:
//                spinHorizontal();
//                break;
//            case VERTICAL:
//                spinVertical();
//                break;
//        }
//    }
//
//    public void setPivotState(PivotState pivotState) {
//        switch (pivotState) {
//            case SCORE:
//                score();
//                break;
//            case SUBMERSIBLE_DOWN:
//                submersibleDown();
//                break;
//            case SUBMERSIBLE_UP:
//                submersibleUp();
//                break;
//            case DROP_OFF:
//                dropOff();
//                break;
//        }
//    }
}
