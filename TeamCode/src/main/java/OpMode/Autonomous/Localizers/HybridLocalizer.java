package OpMode.Autonomous.Localizers;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HybridLocalizer {

    private AprilTagLocalizer aprilTagLocalizer;  // AprilTag localizer for detecting tags
    private Pose fusedPose;  // The final fused pose using Kalman filter

    private KalmanFilter kalmanFilter;

    /**
     * Constructor for the Hybrid Localizer.
     *
     * @param hardwareMap The hardware map to initialize the AprilTag localizer.
     */
    public HybridLocalizer(HardwareMap hardwareMap) {
        this.aprilTagLocalizer = new OpMode.Autonomous.Localizers.AprilTagLocalizer(hardwareMap);
        this.fusedPose = new Pose(0, 0, 0);  // Initialize fused pose
        this.kalmanFilter = new KalmanFilter();
    }

    /**
     * Updates the localization by fusing the Pedro Pathing pose and AprilTag pose.
     */
    public void updateLocalization(Pose followerPose) {


        // Get the current pose from Pedro Pathing and AprilTag localizer
        Pose pedroPose = followerPose;  // Retrieve pose from Pedro Pathing
        AprilTagLocalizer.AprilTagPose aprilTagPose = aprilTagLocalizer.getPose();  // Retrieve pose from AprilTag

        // Convert the AprilTagPose to a Pose2d for consistency
        Pose aprilTagPose2d = new Pose(aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getHeading());

        // Fuse the poses using Kalman filter
        fusedPose = kalmanFilter.fuse(pedroPose, aprilTagPose2d);
    }

    /**
     * Returns the fused pose.
     *
     * @return The fused Pose2d.
     */
    public Pose getFusedPose() {
        return fusedPose;
    }

    /**
     * Stops the vision processing when the localizer is no longer needed.
     */
    public void stop() {
        aprilTagLocalizer.stop();  // Stop the AprilTag localizer
    }

    /**
     * Kalman Filter for fusing the poses.
     */
    private static class KalmanFilter {

        // Kalman filter state variables
        private double[] x = new double[3];  // [x, y, heading]
        private double[][] P = new double[3][3];  // Covariance matrix

        private double R = 0.1;  // Measurement noise (AprilTag Pose)
        private double Q = 0.1;  // Process noise (Pedro Pose)

        public KalmanFilter() {
            // Initialize covariance matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    P[i][j] = 0.1;  // Initial estimate covariance
                }
            }
        }

        /**
         * Fuses the Pedro and AprilTag poses.
         *
         * @param pedroPose    Pose from Pedro Pathing.
         * @param aprilTagPose Pose from AprilTag localization.
         * @return The fused Pose2d.
         */
        public Pose fuse(Pose pedroPose, Pose aprilTagPose) {
            // Prediction step (assuming Pedro Pose is the prediction model)
            double[] z = new double[]{pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading()};
            double[] measurement = new double[]{aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getHeading()};

            // Innovation
            double[] y = new double[3];
            for (int i = 0; i < 3; i++) {
                y[i] = measurement[i] - z[i];
            }

            // Kalman Gain
            double[][] K = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    K[i][j] = P[i][j] / (P[i][i] + R);
                }
            }

            // Update estimate with innovation
            for (int i = 0; i < 3; i++) {
                x[i] = x[i] + K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
            }

            // Update covariance matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    P[i][j] = P[i][j] - K[i][j] * P[j][i];
                }
            }

            // Return the fused pose
            return new Pose(x[0], x[1], x[2]);
        }
    }
}