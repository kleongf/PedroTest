package tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import vision.Detector;

@TeleOp(name = "Detector Test", group = "Sensor")
public class DetectorTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        Detector detector = new Detector();

        waitForStart();

        while (opModeIsActive()) {
            detector.loop();
            telemetry.addData("x-distance", detector.getPositionX());
            telemetry.addData("x-distance", detector.getPositionY());
        }
    }
}
