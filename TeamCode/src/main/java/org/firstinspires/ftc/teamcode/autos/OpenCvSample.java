package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Control;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


@Autonomous
public class OpenCvSample extends LinearOpMode {
    OpenCvWebcam webcam;
    SamplePipeline pipeline;
    @Override
    public void runOpMode() {

        pipeline = new SamplePipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        //webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            /*@Override
            public void onError(int errorCode) {

            }*/
        });

        telemetry.addLine("camera initialized");

        waitForStart();

        while (opModeIsActive()) {
            OpenCvTelemetry(webcam);
            telemetry.addData("avg1", pipeline.getAvg1());
            telemetry.addData("avg2", pipeline.getAvg2());
        }
    }


    class SamplePipeline extends OpenCvPipeline {

        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);

        int detectionThreshold = 120;

        Point r1_point1 = new Point(0, 120);
        Point r1_point2 = new Point(100, 240);

        Point r2_point1 = new Point(180, 120);
        Point r2_point2 = new Point(320, 240);

        Mat region1_Cr, region2_Cr;
        Mat YCrCb = new Mat();
        Mat Cr = new Mat();
        int avg1, avg2;

        void inputToCr(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cr, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCr(firstFrame);

            region1_Cr = Cr.submat(new Rect(r1_point1, r1_point2));
            region2_Cr = Cr.submat(new Rect(r2_point1, r2_point2));
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input, r1_point1, r1_point2, BLUE, 2);
            Imgproc.rectangle(input, r2_point1, r2_point2, BLUE, 2);
            inputToCr(input);

            avg1 = (int) Core.mean(region1_Cr).val[0];
            avg2 = (int) Core.mean(region2_Cr).val[0];

            if (inRegion1()) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        r1_point1, // First point which defines the rectangle
                        r1_point2, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1);
            } else if (inRegion2()) {
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        r2_point1, // First point which defines the rectangle
                        r2_point2, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1);
            }
            return input;
        }

        public int getAvg1() {
            return avg1;
        }

        public int getAvg2() {
            return avg2;
        }

        public boolean inRegion1() {
            return avg1<detectionThreshold;
        }

        public boolean inRegion2() {
            return avg2<detectionThreshold;
        }
    }

    public void OpenCvTelemetry(OpenCvWebcam webcam) {
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
}
