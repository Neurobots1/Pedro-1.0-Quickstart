package OpMode.TeleOp.Test;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

import OpMode.Autonomous.Localizers.AprilTagLocalizer;
import com.pedropathing.localization.Pose;
import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColorAndDistance;
import OpMode.Subsystems.HandServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServosNEW;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "AprilTag Pose Telemetry (OpMode)")
public class AprilTagPoseTelemetryOpMode extends OpMode {

    private AprilTagLocalizer aprilTagLocalizer;
    private Follower follower;
    private final Pose startPose = new Pose(137, 40, Math.toRadians(90));

    @Override
    public void init() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);
        telemetry.addLine("AprilTag Localizer Initialized. Waiting for start...");
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

    }

    @Override
    public void start() {
        // You could reset or do anything special on start if needed
    }

    @Override
    public void loop() {
        follower.update();
        // Update AprilTag localization
        aprilTagLocalizer.updateLocalization();

        // Get current pose
        AprilTagLocalizer.AprilTagPose pose = aprilTagLocalizer.getPose();

        // Display telemetry
        Pose currentPose = follower.getPose();
        telemetry.addData("OTOS X", currentPose.getX());
        telemetry.addData("OTOS Y", currentPose.getY());
        telemetry.addData("OTOS Heading (Degrees)", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("Original Pose",aprilTagLocalizer.originalPose);

        telemetry.addData("Pedro Pose X", aprilTagLocalizer.pedroPose);
        telemetry.update();
    }

    @Override
    public void stop() {
        aprilTagLocalizer.stop();
    }
}
