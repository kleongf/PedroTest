package opmodes.teleop;

import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import robot.Alliance;
import robot.Robot;
import robot.OpModeType;

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    Robot robot;
    private final Pose startPose = new Pose(123.5, 18.5, Math.toRadians(135));

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1 , gamepad2, telemetry, Alliance.RED, OpModeType.TELEOP, startPose);
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.updateControls();
        robot.loop();
    }
}
