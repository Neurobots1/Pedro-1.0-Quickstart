package OpMode.Autonomous.Localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

/**
 * Independent AprilTag Localizer that calculates and returns its own pose based on AprilTag detections.
 */
public class AprilTagLocalizer {

    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private AprilTagPose currentPose = new AprilTagPose(0, 0, 0);  // Default pose: origin (0,0) and 0 rotation

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 12, 0);  // Adjust position as needed
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    /**
     * Constructor for the AprilTag Localizer.
     *
     * @param hardwareMap The HardwareMap to access robot hardware.
     */
    public AprilTagLocalizer(HardwareMap hardwareMap) {
        initAprilTagProcessor(hardwareMap);
    }

    /**
     * Initializes the AprilTag Processor and Vision Portal.
     */
    private void initAprilTagProcessor(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();
        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        portalBuilder.addProcessor(aprilTagProcessor);

        visionPortal = portalBuilder.build();
    }

    /**
     * Updates the robot's pose based on AprilTag detections.
     */
    public void updateLocalization() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (!detections.isEmpty()) {
            // Use the first detection for localization
            AprilTagDetection detection = detections.get(0);
            AprilTagPose tagPose = new AprilTagPose(
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            );

            currentPose = tagPose;  // Update the current pose based on the AprilTag detection
        }
    }

    /**
     * Returns the current pose of the robot.
     *
     * @return The current pose as an AprilTagPose.
     */
    public AprilTagPose getPose() {
        return currentPose;
    }

    /**
     * Stops the vision processing when the localizer is no longer needed.
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();  // Close the vision portal to release resources
        }
    }

    /**
     * A custom class for representing a pose, independent of the Pedro Pathing Pose2d.
     */
    public static class AprilTagPose {
        private double x;
        private double y;
        private double rotation;

        public AprilTagPose(double x, double y, double rotation) {
            this.x = x;
            this.y = y;
            this.rotation = rotation;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getRotation() {
            return rotation;
        }
    }
}
