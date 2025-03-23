package OpMode.Autonomous.Localizers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import OpMode.Autonomous.Localizers.AprilTagLocalizer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "CameraOffsetFinder", group = "Active")
public class CameraOffsetFinder extends OpMode {

    private Follower follower;
    private AprilTagLocalizer aprilTagLocalizer;

    private final Pose startPose = new Pose(7, 104, Math.toRadians(270));

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
    }

    @Override
    public void loop() {
        // Update robot movement
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        // Get the current pose from Pedro Pathing
        Pose pedroPose = follower.getPose();

        // Get the AprilTag-detected pose
        AprilTagLocalizer.AprilTagPose cameraPose = aprilTagLocalizer.getPose();

        // Display Pedro Pathing Pose
        telemetry.addData("\nPedro Pathing Pose", "");
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());
        telemetry.addData("Heading (Degrees)", Math.toDegrees(pedroPose.getHeading()));

        // Display AprilTag Camera Pose
        if (cameraPose != null) {
            telemetry.addData("\nAprilTag Camera Pose", "");
            telemetry.addData("X", cameraPose.getX());
            telemetry.addData("Y", cameraPose.getY());
            telemetry.addData("Heading (Degrees)", Math.toDegrees(cameraPose.getRotation()));

            // Calculate offsets (AprilTag Pose - Pedro Pathing Pose)
            double xOffset = cameraPose.getX() - pedroPose.getX();
            double yOffset = cameraPose.getY() - pedroPose.getY();
            double headingOffset = cameraPose.getRotation() - pedroPose.getHeading();

            // Display Calculated Camera Offsets
            telemetry.addData("\nCamera Offsets", "");
            telemetry.addData("X Offset", xOffset);
            telemetry.addData("Y Offset", yOffset);
            telemetry.addData("Heading Offset (Degrees)", Math.toDegrees(headingOffset));
        } else {
            telemetry.addData("AprilTag", "Not detected");
        }

        telemetry.update();
    }
}
