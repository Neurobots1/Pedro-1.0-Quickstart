package OpMode.Autonomous.Localizers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.ejml.dense.block.MatrixMult_FDRB;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class OpenCVTest extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

       webcam1.setPipeline(new exemplePipeline());

       webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
           public void onOpened() {
            webcam1.startStreaming(640, 360, OpenCvCameraRotation.SIDEWAYS_RIGHT);
           }

           public void onError(int errorCode) {

           }
       });

    }

    @Override
    public void loop() {

    }

    class exemplePipeline extends OpenCvPipeline{
       Mat YCbCr = new Mat();
       Mat leftCrop;
       Mat rightCrop;
       double leftavgfin;
       double rightavgfin;
       Mat output = new Mat();
       Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


        private Mat preprocessFrame(Mat frame){

            Imgproc.cvtColor(frame,YCbCr,Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Mat redmask = new Mat();

            Rect leftRect = new Rect(1, 1, 319, 359);
            Rect rightRect = new Rect(1, 1, 319, 359);

            frame.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(leftCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            return (redmask);
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
    }

        @Override
        public Mat processFrame(Mat input) {

            Mat redMask = preprocessFrame(input);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);

            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

            return input;
        }
    }}