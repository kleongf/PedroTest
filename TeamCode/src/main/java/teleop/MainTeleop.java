package teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import shared.Extend;
import shared.Intake;
import shared.Lift;

@TeleOp(name = "Main TeleOp")
public class MainTeleop extends OpMode {

    public enum LiftState {
        LIFT_START,
        LIFT_UP_HIGH,
        LIFT_UP_LOW,
        EXTEND_RETRACT,
        LIFT_DOWN,
        WAIT,
    }

    public enum IntakeState {
        INTAKE_START,
        INTAKE_UP,
        INTAKE_DOWN,
        INTAKE_REVERSE
    }

    LiftState liftState = LiftState.LIFT_START;
    IntakeState intakeState = IntakeState.INTAKE_START;

    public DcMotorEx liftMotorOne;
    public DcMotorEx liftMotorTwo;
    public DcMotorEx extendMotorOne;
    public DcMotorEx extendMotorTwo;
    public Servo rotateMotorOne;
    public Servo rotateMotorTwo;
    public CRServo intakeMotor;
    private AnalogInput analogEncoder;

    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    Lift lift;
    Extend extend;
    Intake intake;

    public int LIFT_UP = 240;
    public int LIFT_ZERO = 150;
    public int LIFT_SAMPLE = 145;

    public int EXTEND_HIGH = 700;
    public int EXTEND_LOW = 300;
    public int EXTEND_ZERO = 0;

    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    private boolean rightBumperPressed = false;
    private boolean leftStickPressed = false;
    private boolean rightStickPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftBumperPressed = false;

    public void init() {
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "liftMotorOne");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "liftMotorTwo");
        analogEncoder = hardwareMap.get(AnalogInput.class, "encoder");
        extendMotorOne = hardwareMap.get(DcMotorEx.class, "extendMotorOne");
        extendMotorTwo = hardwareMap.get(DcMotorEx.class, "extendMotorTwo");
        rotateMotorOne = hardwareMap.get(Servo.class, "rotateMotorOne");
        rotateMotorTwo = hardwareMap.get(Servo.class, "rotateMotorTwo");
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "left_front");
        frontRight = hardwareMap.get(DcMotorEx.class, "right_front");
        backLeft = hardwareMap.get(DcMotorEx.class, "left_back");
        backRight = hardwareMap.get(DcMotorEx.class, "right_back");

        lift = new Lift(liftMotorOne, liftMotorTwo, analogEncoder);
        extend = new Extend(extendMotorOne, extendMotorTwo);
        intake = new Intake(rotateMotorOne, rotateMotorTwo, intakeMotor);
        actionTimer.reset();
    }
    /*
    Right bumper: high basket
    Right trigger: low basket
    Left bumper: retract
        if angle > 160, then we do retract first then lift down
        if angle < 160, then we do lift down first then retract
    TODO:
    Left stick button: Intake (forward)
    TODO:
    Right stick button: Outtake (up, reverse, down)
    Left stick: Extension clipping
    Right stick: Angle clipping
    TODO:
    Triangle: hang?
        Triangle will set the extension to zero
        We may need to change PID coefficients to make sure there is enough power
     */

    public void loop() {
        switch (liftState) {
            case LIFT_START:
                if (gamepad1.right_bumper && !rightBumperPressed) {
                    lift.setTarget(LIFT_UP);
                    actionTimer.reset();
                    liftState = LiftState.LIFT_UP_HIGH;
                } else if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
                    lift.setTarget(LIFT_UP);
                    actionTimer.reset();
                    liftState = LiftState.LIFT_UP_LOW;
                } else if (gamepad1.left_bumper && !leftBumperPressed) {
                    if ((((analogEncoder.getVoltage() / 3.2 * 360) + 145) % 360) > 170) {
                        extend.setTarget(EXTEND_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.EXTEND_RETRACT;
                    } else {
                        lift.setTarget(LIFT_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.LIFT_DOWN;
                    }
                }
                break;

            case LIFT_UP_HIGH:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_HIGH);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            case LIFT_UP_LOW:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_LOW);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            case WAIT:
                if (actionTimer.seconds() >= 0.3) {
                    actionTimer.reset();
                    liftState = LiftState.LIFT_START;
                }
                break;

            case LIFT_DOWN:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_ZERO);
                    actionTimer.reset();
                    liftState = LiftState.WAIT;
                }
                break;

            default:
                liftState = LiftState.LIFT_START;
        }

        switch (intakeState) {
            case INTAKE_START:
                if (gamepad1.left_stick_button && !leftStickPressed) {
                    intake.IntakeForward();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                } else if (gamepad1.right_stick_button && !rightStickPressed) {
                    intake.IntakeUp();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_UP;
                }
                break;

            case INTAKE_UP:
                if (intakeTimer.seconds() >= 0.3) {
                    intake.IntakeReverse();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_REVERSE;
                }
                break;

            case INTAKE_REVERSE:
                if (intakeTimer.seconds() >= 0.5) {
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_DOWN;
                }
                break;

            case INTAKE_DOWN:
                if (intakeTimer.seconds() >= 0.3) {
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        // joystick: trying new stuff instead of shart's old stuff
        double armLengthInput = gamepad1.left_stick_y;
        telemetry.addData("left stick", armLengthInput);
        if (Math.abs(armLengthInput) > 0.1) {
//            extendMotorOne.setPower(armLengthInput * 0.9);
//            extendMotorTwo.setPower(armLengthInput * 0.9);

            if (armLengthInput > 0) {
                extend.setTarget(extend.getTarget() + 5);
            } else {
                extend.setTarget(extend.getTarget() - 5);
            }
        }

        double armAngleInput = gamepad1.right_stick_y;
        telemetry.addData("right stick", armAngleInput);
        if (Math.abs(armAngleInput) > 0.1) {
//            liftMotorOne.setPower(armAngleInput * 0.2);
//            liftMotorTwo.setPower(armAngleInput * 0.2);
            if (armLengthInput > 0) {
                lift.setTarget(lift.getTarget() + 0.1);
            } else {
                lift.setTarget(lift.getTarget() - 0.1);
            }
        }

        extend.loop();
        lift.loop();
        intake.loop();

        telemetry.update();

        rightTriggerPressed = gamepad1.right_trigger > 0.5;
        rightBumperPressed = gamepad1.right_bumper;
        leftBumperPressed = gamepad1.left_bumper;
        leftStickPressed = gamepad1.left_stick_button;
        rightStickPressed = gamepad1.right_stick_button;

        // Drive logic (unchanged)
        double y = -this.gamepad2.left_stick_y;
        double x = this.gamepad2.left_stick_x * 1.1;
        double rx = this.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(-frontRightPower);
        backRight.setPower(-backRightPower);
    }
}

