package tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import vision.YCrCbPipelineRY;
import vision.YellowPipeline;

/*
 * This version of the internal camera example uses EasyOpenCV's interface to the
 * original Android camera API
 */
@TeleOp(name = "OpenCV YCrCb Test")
public class OpenCVTest extends LinearOpMode
{
    OpenCvCamera phoneCam;
    YCrCbPipelineRY yellowPipeline;

    public static int lowerYellowY = 50;
    public static int lowerYellowCr = 100;
    public static int lowerYellowCb = 0;

    public static int upperYellowY = 255;
    public static int upperYellowCr = 255;
    public static int upperYellowCb = 100;

    public static int lowerRedY = 50;
    public static int lowerRedCr = 180;
    public static int lowerRedCb = 0;

    public static int upperRedY = 255;
    public static int upperRedCr = 255;
    public static int upperRedCb = 140;

    public static int minArea = 10000;

    public static int X_MIN = 0, X_MAX = 640, Y_MIN = 0, Y_MAX = 480;

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        FtcDashboard.getInstance().startCameraStream(phoneCam, 0);
        yellowPipeline = new YCrCbPipelineRY();
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
            yellowPipeline.setLOWER_YELLOW(lowerYellowY, lowerYellowCr, lowerYellowCb);
            yellowPipeline.setUPPER_YELLOW(upperYellowY, upperYellowCr, upperYellowCb);
            yellowPipeline.setLOWER_RED(lowerRedY, lowerRedCr, lowerRedCb);
            yellowPipeline.setUPPER_RED(upperRedY, upperRedCr, upperRedCb);

            yellowPipeline.setMIN_CONTOUR_AREA(minArea);

            yellowPipeline.setX_MIN(X_MIN);
            yellowPipeline.setX_MAX(X_MAX);
            yellowPipeline.setY_MIN(Y_MIN);
            yellowPipeline.setY_MAX(Y_MAX);

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Object In Center", yellowPipeline.isBlockDetected());
            telemetry.addData("Orientation", yellowPipeline.getOrientation());
            telemetry.update();
        }
    }
}
