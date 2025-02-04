package vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Detector {
    private final Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
    private final double height = 4;
    private double currentX = 0;
    private double currentY = 0;
    private double cameraOffset = 0;

    public double calculateDistanceY(double targetY) {
        // watch out for div by 0 errors
        if (targetY == 0) {
            return 0;
        }
        return (height / (Math.tan(Math.toRadians(-targetY))));
    }

    public double calculateDistanceX(double targetY, double targetX) {
        if (targetY == 0) {
            return 0;
        }
        return (height / (Math.tan(Math.toRadians(-targetY)))) * Math.tan(Math.toRadians(targetX));
    }

    public Detector() {
        telemetry.setMsTransmissionInterval(11);
        // pipeline 1 is python snapscript
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void loop() {
        LLResult result = limelight.getLatestResult();
        // we are getting python results
        if (result != null) {
            if (result.isValid()) {
                currentX = result.getTx();
                currentY = result.getTy();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
            currentX = 0;
            currentY = 0;
        }
        telemetry.update();
    }

    public double getPositionX() {
        if (currentX == 0 && currentY == 0) {
            return 0;
        } else {
            return calculateDistanceX(currentY, currentX) + cameraOffset;
        }
    }

    public double getPositionY() {
        if (currentX == 0 && currentY == 0) {
            return 0;
        } else {
            return calculateDistanceY(currentY);
        }
    }
}
