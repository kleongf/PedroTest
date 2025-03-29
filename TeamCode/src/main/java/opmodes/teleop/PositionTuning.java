package opmodes.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import robot.subsystems.ExtendSubsystem;
import robot.subsystems.IntakeSubsystem;
import robot.subsystems.LiftSubsystem;

@Config
@TeleOp
public class PositionTuning extends OpMode {

    public ExtendSubsystem extend;
    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public static int extendTarget = 0;
    public static int liftTarget = 59;
    public static double intakeTarget = 0.3;

    public static double spinTarget = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extend = new ExtendSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        lift.setTarget(liftTarget);
        extend.setTarget(extendTarget);
        intake.setTarget(intakeTarget);
        intake.spin(spinTarget);
        lift.loop();
        extend.loop();
        telemetry.update();
    }
}

