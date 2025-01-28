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
        EXTEND_RETRACT,
        LIFT_DOWN,
        WAIT,
    }

    public enum IntakeState {
        INTAKE_START,
        INTAKE_UP,
        INTAKE_DOWN,
        INTAKE_REVERSE,
        INTAKE_LIFT_DOWN
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

    public int LIFT_UP = 180;
    public int LIFT_ZERO = 92;
    public int LIFT_OUT = 92;

    public int EXTEND_HIGH = 700;
    public int EXTEND_ZERO = 0;
    public int EXTEND_MID = 350;
    public int EXTEND_MAX = 700;

    ElapsedTime actionTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();

    private boolean leftStickPressed = false;
    private boolean rightStickPressed = false;
    private boolean rightTriggerPressed = false;
    private boolean leftBumperPressed = false;
    private boolean crossPressed = false;
    private boolean leftTriggerPressed = false;
    private boolean rightBumperPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

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
    Lt mid distance (done)
    LB max distance out (done)
    LSB intake running and intake flip down (done)
    RSB eject (done)

    D-pad up get ready for climb (need to test climb first)
    D-pad down Climb (need to test climb first)
    A button retract and flip up intake (done, also works for retracting when grabbing sample)

    Righgt trigger high bucket (done)
    Right button specmin place (can't do specimen rn, skipping)

    Right bumper: lift down to block
     */

    public void loop() {
        switch (liftState) {
            case LIFT_START:
                if (gamepad1.right_trigger > 0.5 && !rightTriggerPressed) {
                    lift.setTarget(LIFT_UP);
                    actionTimer.reset();
                    liftState = LiftState.LIFT_UP_HIGH;
                } else if (gamepad1.cross && !crossPressed) {
                    if (lift.getTarget() > LIFT_ZERO + 20) {
                        extend.setTarget(EXTEND_ZERO);
                        actionTimer.reset();
                        liftState = LiftState.EXTEND_RETRACT;
                    } else {
                        lift.setTarget(LIFT_OUT);
                        actionTimer.reset();
                        liftState = LiftState.LIFT_DOWN;
                    }
                } else if (gamepad1.left_trigger > 0.5 && !leftTriggerPressed) {
                    extend.setTarget(EXTEND_MID);
                } else if (gamepad1.left_bumper && !leftBumperPressed) {
                    extend.setTarget(EXTEND_MAX);
                }
                break;

            case LIFT_UP_HIGH:
                if (actionTimer.seconds() >= 0.3) {
                    extend.setTarget(EXTEND_HIGH);
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

            case EXTEND_RETRACT:
                if (actionTimer.seconds() >= 0.3) {
                    lift.setTarget(LIFT_ZERO);
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
                    intake.IntakeDown();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_LIFT_DOWN;
                } else if (gamepad1.right_stick_button && !rightStickPressed) {
                    intake.IntakeReverse();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            case INTAKE_LIFT_DOWN:
                if (intakeTimer.seconds() >= 0.3) {
                    if (extend.getTarget() <= 350) {
                        lift.setTarget(89);
                    } else if (extend.getTarget() <= 700) {
                        lift.setTarget(91);
                    } else {
                        lift.setTarget(88 + 5 * (extend.getTarget() / 700.0));
                    }
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_UP;
                }
                break;

            case INTAKE_UP:
                if (!gamepad1.left_stick_button) {
                    intake.IntakeUp();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

//            case INTAKE_REVERSE:
//                if (intakeTimer.seconds() >= 0.5) {
//                    intakeTimer.reset();
//                    intakeState = IntakeState.INTAKE_DOWN;
//                }
//                break;
//
//            case INTAKE_DOWN:
//                if (intakeTimer.seconds() >= 0.3) {
//                    intake.IntakeDown();
//                    intakeTimer.reset();
//                    intakeState = IntakeState.INTAKE_START;
//                }
//                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        // dpad trimming
        if (gamepad1.dpad_up && !dpadUpPressed) {
            extend.setTarget(extend.getTarget() + 30);
        } else if (gamepad1.dpad_down && !dpadDownPressed) {
            extend.setTarget(extend.getTarget() - 30);
        }

        extend.loop();
        lift.loop();
        intake.loop();

        telemetry.update();

        rightTriggerPressed = gamepad1.right_trigger > 0.5;
        leftBumperPressed = gamepad1.left_bumper;
        leftStickPressed = gamepad1.left_stick_button;
        rightStickPressed = gamepad1.right_stick_button;
        leftTriggerPressed = gamepad1.left_trigger > 0.5;
        crossPressed = gamepad1.cross;
        rightBumperPressed = gamepad1.right_bumper;
        dpadDownPressed = gamepad1.dpad_down;
        dpadUpPressed = gamepad1.dpad_up;

        // Drive logic (unchanged)
        double y = -this.gamepad2.left_stick_y;
        double x = this.gamepad2.left_stick_x * 1.1;
        double rx = this.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        boolean lowPower = gamepad2.left_trigger > 0.5;

        frontLeft.setPower(lowPower ? 0.5 * frontLeftPower : frontLeftPower);
        backLeft.setPower(lowPower ? 0.5 * backLeftPower : backLeftPower);
        frontRight.setPower(lowPower ? 0.5 * -frontRightPower : -frontRightPower);
        backRight.setPower(lowPower ? 0.5 * -backRightPower : -backRightPower);
    }
}

