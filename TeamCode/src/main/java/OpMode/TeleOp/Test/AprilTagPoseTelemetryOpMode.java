package OpMode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import OpMode.Autonomous.Localizers.AprilTagLocalizer;

@TeleOp(name = "AprilTag Pose Telemetry (OpMode)")
public class AprilTagPoseTelemetryOpMode extends OpMode {

    private AprilTagLocalizer aprilTagLocalizer;

    @Override
    public void init() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        telemetry.addLine("AprilTag Localizer Initialized. Waiting for start...");
    }

    @Override
    public void start() {
        // You could reset or do anything special on start if needed
    }

    @Override
    public void loop() {
        // Update AprilTag localization
        aprilTagLocalizer.updateLocalization();

        // Get current pose
        AprilTagLocalizer.AprilTagPose pose = aprilTagLocalizer.getPose();

        // Display telemetry
        telemetry.addData("Pose X (inches)", pose.getX());
        telemetry.addData("Pose Y (inches)", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagLocalizer.stop();
    }
}
