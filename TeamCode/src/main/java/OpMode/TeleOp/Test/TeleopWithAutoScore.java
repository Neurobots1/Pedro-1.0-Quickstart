package OpMode.TeleOp.Test;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


/**
 * Teleop with automatic scoring using existing odometry (No Odometry Reset).
 */
@TeleOp(name = "Teleop with Auto Scoring (No Sensors)", group = "Examples")
public class TeleopWithAutoScore extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose bucketPose = new Pose(11, 128, Math.toRadians(315)); // Target scoring pose

    private boolean isAuto = false;
    private boolean hasScored = false;

    /** This method is called once when init is pressed **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    /** This method is called once at the start of the OpMode **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if (!isAuto) {
            // Normal TeleOp movement
            follower.setTeleOpMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            // Press "A" to start the scoring sequence
            if (gamepad1.a) {
                startScoringSequence();
            }
        } else {
            // If in auto mode and robot has finished moving, proceed to scoring
            if (!follower.isBusy()) {
                scoreItem();
            }
        }

        follower.update();

        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Is Auto Aligning", isAuto);
        telemetry.addData("Has Scored", hasScored);
        telemetry.update();
    }

    private void startScoringSequence() {
        isAuto = true;
        hasScored = false;

        // Move to the bucketPose
        PathChain moveToBucket = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(bucketPose)
                ))
                .setConstantHeadingInterpolation(bucketPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.2)
                .build();

        follower.followPath(moveToBucket);
    }

    private void scoreItem() {
        // Simulated scoring action (e.g., opening a claw)
        hasScored = true;

        // Exit auto mode and return to manual control
        isAuto = false;
    }

    /** This method stops the robot **/
    @Override
    public void stop() {
    }
}
