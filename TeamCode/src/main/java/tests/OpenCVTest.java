package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import vision.YellowPipeline;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@TeleOp
public class OpenCVTest extends LinearOpMode
{
    OpenCvCamera phoneCam;
    YellowPipeline yellowPipeline;

    @Override
    public void runOpMode()
    {
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        YellowPipeline yellowPipeline = new YellowPipeline();
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.setPipeline(yellowPipeline);
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Error");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("Object In Center", yellowPipeline.isBlockDetected());
            telemetry.update();
        }
    }
}
