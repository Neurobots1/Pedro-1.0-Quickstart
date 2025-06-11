package OpMode.Autonomous.Localizers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "camera")
public class camera extends LinearOpMode {

    private OpenCvCamera controlHubCam;
    private VisionPipeline pipeline;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        initOpenCV();

        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        waitForStart();

        while (opModeIsActive()) {
            double dist = getDistance(pipeline.getWidth());
            telemetry.addData("Distance in Inch", dist);
            telemetry.addData("Angle", pipeline.getAngle());
            telemetry.update();
            sleep(50);
        }

        controlHubCam.stopStreaming();
        dashboard.stopCameraStream();
    }

    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new VisionPipeline();
        controlHubCam.setPipeline(pipeline);

        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                controlHubCam.startStreaming(VisionPipeline.camWidth, VisionPipeline.camHeight, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                telemetry.addData("Camera", "Opened and Streaming");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera open error", errorCode);
                telemetry.update();
            }
        });
    }

    class VisionPipeline extends OpenCvPipeline {

        private double width = 0;
        private double angle = 0;

        double X = 0;
        double Y = 0;

        static final int camWidth = 640;
        static final int camHeight = 480;

        private static final double ObjectRealWide = 1.5;
        private static final double FocalLength = 1980;

        Scalar lineColor = new Scalar(0, 255, 0);
        int lineThickness = 3;

        Scalar lowerHSV = new Scalar(22, 98, 101);
        Scalar upperHSV = new Scalar(28, 255, 255);

        Mat hsvMat = new Mat();
        Mat hsvBinaryMat = new Mat();

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        MatOfPoint biggestContour = null;

        Mat inputContours = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

            contours.clear();
            hierarchy.release();
            Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            biggestContour = null;
            for (MatOfPoint contour : contours) {
                if (biggestContour == null || Imgproc.contourArea(contour) > Imgproc.contourArea(biggestContour)) {
                    biggestContour = contour;
                    width = calculateWidth(contour);
                    angle = calculateAngleAndDisplay(input, contour);
                }
            }

            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(X + 10, Y + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(X + 10, Y + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            Moments moments = Imgproc.moments(biggestContour);
            X = moments.get_m10() / moments.get_m00();
            Y = moments.get_m01() / moments.get_m00();

            String label = "(" + (int) X + ", " + (int) Y + ")";
            Imgproc.putText(input, label, new Point(X + 10, Y), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(X, Y), 5, new Scalar(0, 255, 0), -1);

            input.copyTo(inputContours);

            if (biggestContour != null) {
                ArrayList<MatOfPoint> contoursList = new ArrayList<>();
                contoursList.add(biggestContour);
                Imgproc.drawContours(inputContours, contoursList, -1, lineColor, lineThickness);
            } else {
                width = 0;
                angle = 0;
            }

            return inputContours;
        }

        private double calculateWidth(MatOfPoint contour) {
            if (contour == null) return 0;
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

        private double calculateAngleAndDisplay(Mat img, MatOfPoint contour) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            double width = rect.size.width;
            double height = rect.size.height;
            double angle = rect.angle;

            if (width < height) {
                angle = 90 + angle;
            }

            Point center = rect.center;
            Imgproc.putText(img,
                    String.format("Angle: %.1f", angle),
                    new Point(center.x - 30, center.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    new Scalar(255, 255, 255),
                    2);

            return angle;
        }

        public double getAngle() {
            return angle;
        }

        public double getWidth() {
            return width;
        }
    }

    private double getDistance(double width){
        double distance = (VisionPipeline.ObjectRealWide * VisionPipeline.FocalLength) / width;
        return distance;
    }
}
