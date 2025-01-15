package OpMode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ExtendoServos;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutonomousTest", group = "Autonomous")
public class AutonomousTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ViperSlides viperSlides;
    private IntakeMotor intakeMotor;

    // Servo Subsystems
    private BucketServos bucketServos;
    private ClawServo clawServo;
    private ExtendoServos extendoServos;
    private IntakeServos intakeServos;

    private int pathState;


    // Define Poses and Paths
    private final Pose startPose = new Pose(8, 80, Math.toRadians(270));
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));
    private final Pose pickup1Pose = new Pose(28, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(35, 130, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(30, 130, Math.toRadians(35));
    private final Pose parkPose = new Pose(14, 129, Math.toRadians(315));
    //Control Points
    private final Pose parkControlPose = new Pose(64, 130);

    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */

                    // Set the Viper slides to the HIGH target position
                    viperSlides.setTarget(ViperSlides.Target.HIGH);

                    // Wait until the Viper slides reach the HIGH target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.HIGH)) {
                        // Keep checking if the slides are at the target
                        // Optionally, you can add telemetry or logging here for monitoring
                    }

                    // Once the slides are at the HIGH position, move the bucket servos to the deposit position
                    bucketServos.depositPosition();

                    // Wait until the bucket servos reach the deposit position
                    while (!bucketServos.isDepositPosition()) {
                        // Keep checking if the bucket servos are at the deposit position
                    }

                    // Once the bucket servos are at the deposit position, move them to the transfer position
                    bucketServos.transferPosition();

                    // Wait until the bucket servos are in the transfer position
                    while (!bucketServos.isTransferPosition()) {
                        // Keep checking if the bucket servos are at the transfer position
                    }

                    // Finally, set the Viper slides to the LOW target position
                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    // Optionally, wait for the Viper slides to reach the LOW target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.LOW)) {
                        // Keep checking if the slides are at the LOW position
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (follower.getPose().getX() > (pickup1Pose.getX() - 1) && follower.getPose().getY() > (pickup1Pose.getY() - 1)) {
                    /* Grab Sample */

                    actionTimer.resetTimer(); // Reset the action timer before starting the process

                    // Extend the extendo servos
                    extendoServos.extend();

                    // Wait until extendo servos are extended
                    while (!extendoServos.isExtended()) {
                        // Do nothing, just wait
                    }

                    // Set intake servos to the intaking position
                    intakeServos.intakePosition();

                    // Wait until intake servos are in the intake position
                    while (!intakeServos.isIntakePosition()) {
                        // Do nothing, just wait
                    }

                    // Start running intake motor to intaking
                    intakeMotor.intake();
                    actionTimer.resetTimer(); // Reset the action timer before starting the intake process

                    // Continue running intake motor for 5 seconds, check the elapsed time
                    while (actionTimer.getElapsedTimeSeconds() < 5) {
                        // Do other tasks if needed, the program continues running
                    }

                    // Stop intake motor after 5 seconds
                    intakeMotor.stop();

                    // Set intake servos to the transfer position
                    intakeServos.transferPosition();

                    // Run intake motor at keep intaking while moving the intake servos
                    intakeMotor.keepIntaking();

                    // Wait until intake servos are at the transfer position
                    while (!intakeServos.isTransferPosition()) {
                        // Do nothing, just wait
                    }

                    // Stop intake motor
                    intakeMotor.stop();

                    // Retract the extendo servos
                    extendoServos.retract();

                    // Wait until extendo servos are retracted
                    while (!extendoServos.isRetracted()) {
                        // Do nothing, just wait
                    }

                    // Run intake motor to outtake
                    intakeMotor.outtake();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */

                    // Set the Viper slides to the HIGH target position
                    viperSlides.setTarget(ViperSlides.Target.HIGH);

                    // Wait until the Viper slides reach the HIGH target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.HIGH)) {
                        // Keep checking if the slides are at the target
                        // Optionally, you can add telemetry or logging here for monitoring
                    }

                    // Once the slides are at the HIGH position, move the bucket servos to the deposit position
                    bucketServos.depositPosition();

                    // Wait until the bucket servos reach the deposit position
                    while (!bucketServos.isDepositPosition()) {
                        // Keep checking if the bucket servos are at the deposit position
                    }

                    // Once the bucket servos are at the deposit position, move them to the transfer position
                    bucketServos.transferPosition();

                    // Wait until the bucket servos are in the transfer position
                    while (!bucketServos.isTransferPosition()) {
                        // Keep checking if the bucket servos are at the transfer position
                    }

                    // Finally, set the Viper slides to the LOW target position
                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    // Optionally, wait for the Viper slides to reach the LOW target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.LOW)) {
                        // Keep checking if the slides are at the LOW position
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (follower.getPose().getX() > (pickup2Pose.getX() - 1) && follower.getPose().getY() > (pickup2Pose.getY() - 1)) {
                    /* Grab Sample */

                    actionTimer.resetTimer(); // Reset the action timer before starting the process
                    // Extend the extendo servos
                    extendoServos.extend();

                    // Wait until extendo servos are extended
                    while (!extendoServos.isExtended()) {
                        // Do nothing, just wait
                    }

                    // Set intake servos to intake position
                    intakeServos.intakePosition();

                    // Wait until intake servos are in intake position
                    while (!intakeServos.isIntakePosition()) {
                        // Do nothing, just wait
                    }

                    // Run intake motor at intaking for 5 seconds
                    intakeMotor.intake();
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTimeSeconds() < 5) {
                        // Do nothing, just wait for 5 seconds
                    }
                    intakeMotor.stop();

                    // Set intake servos to transfer position and keep running intake motor
                    intakeServos.transferPosition();
                    intakeMotor.keepIntaking();

                    // Wait until intake servos are in transfer position
                    while (!intakeServos.isTransferPosition()) {
                        // Do nothing, just wait
                    }

                    // Retract the extendo servos
                    extendoServos.retract();

                    // Wait until extendo servos are retracted
                    while (!extendoServos.isRetracted()) {
                        // Do nothing, just wait
                    }

                    // Run intake motor at outtake
                    intakeMotor.outtake();


                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */

                    // Set the Viper slides to the HIGH target position
                    viperSlides.setTarget(ViperSlides.Target.HIGH);

                    // Wait until the Viper slides reach the HIGH target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.HIGH)) {
                        // Keep checking if the slides are at the target
                        // Optionally, you can add telemetry or logging here for monitoring
                    }

                    // Once the slides are at the HIGH position, move the bucket servos to the deposit position
                    bucketServos.depositPosition();

                    // Wait until the bucket servos reach the deposit position
                    while (!bucketServos.isDepositPosition()) {
                        // Keep checking if the bucket servos are at the deposit position
                    }

                    // Once the bucket servos are at the deposit position, move them to the transfer position
                    bucketServos.transferPosition();

                    // Wait until the bucket servos are in the transfer position
                    while (!bucketServos.isTransferPosition()) {
                        // Keep checking if the bucket servos are at the transfer position
                    }

                    // Finally, set the Viper slides to the LOW target position
                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    // Optionally, wait for the Viper slides to reach the LOW target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.LOW)) {
                        // Keep checking if the slides are at the LOW position
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3, true);
                    setPathState(6);
                }
                break;
            case 6:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (follower.getPose().getX() > (pickup3Pose.getX() - 1) && follower.getPose().getY() > (pickup3Pose.getY() - 1)) {
                    /* Grab Sample */

                    actionTimer.resetTimer(); // Reset the action timer before starting the process

                    // Extend the extendo servos
                    extendoServos.extend();

                    // Wait until extendo servos are extended
                    while (!extendoServos.isExtended()) {
                        // Do nothing, just wait
                    }

                    // Set intake servos to intake position
                    intakeServos.intakePosition();

                    // Wait until intake servos are in intake position
                    while (!intakeServos.isIntakePosition()) {
                        // Do nothing, just wait
                    }

                    // Run intake motor at intaking for 5 seconds
                    intakeMotor.intake();
                    actionTimer.resetTimer();
                    while (actionTimer.getElapsedTimeSeconds() < 5) {
                        // Do nothing, just wait for 5 seconds
                    }
                    intakeMotor.stop();

                    // Set intake servos to transfer position and keep running intake motor
                    intakeServos.transferPosition();
                    intakeMotor.keepIntaking();

                    // Wait until intake servos are in transfer position
                    while (!intakeServos.isTransferPosition()) {
                        // Do nothing, just wait
                    }

                    // Retract the extendo servos
                    extendoServos.retract();

                    // Wait until extendo servos are retracted
                    while (!extendoServos.isRetracted()) {
                        // Do nothing, just wait
                    }

                    // Run intake motor at outtake
                    intakeMotor.outtake();  // Corrected spelling of "outtake"



                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */

                    // Set the Viper slides to the HIGH target position
                    viperSlides.setTarget(ViperSlides.Target.HIGH);

                    // Wait until the Viper slides reach the HIGH target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.HIGH)) {
                        // Keep checking if the slides are at the target
                        // Optionally, you can add telemetry or logging here for monitoring
                    }

                    // Once the slides are at the HIGH position, move the bucket servos to the deposit position
                    bucketServos.depositPosition();

                    // Wait until the bucket servos reach the deposit position
                    while (!bucketServos.isDepositPosition()) {
                        // Keep checking if the bucket servos are at the deposit position
                    }

                    // Once the bucket servos are at the deposit position, move them to the transfer position
                    bucketServos.transferPosition();

                    // Wait until the bucket servos are in the transfer position
                    while (!bucketServos.isTransferPosition()) {
                        // Keep checking if the bucket servos are at the transfer position
                    }

                    // Finally, set the Viper slides to the LOW target position
                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    // Optionally, wait for the Viper slides to reach the LOW target position
                    while (!viperSlides.isAtTargetPosition(ViperSlides.Target.LOW)) {
                        // Keep checking if the slides are at the LOW position
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (parkPose.getX() - 1) && follower.getPose().getY() > (parkPose.getY() - 1)) {
                    /* Level 1 Ascent */
                    viperSlides.setTarget(ViperSlides.Target.LEVEL1);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Initialize ViperSlides
        DcMotorEx slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");
        DcMotorEx slideMotorRight = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        viperSlides = new ViperSlides(slideMotorLeft, slideMotorRight, limitSwitch, 0.01, 0.0, 0.005);

        // Initialize Bucket Servos
        bucketServos = new BucketServos(
                hardwareMap.get(Servo.class, "BucketServoRight"),
                hardwareMap.get(Servo.class, "BucketServoLeft")
        );

        // Initialize Claw Servo
        clawServo = new ClawServo(
                hardwareMap.get(Servo.class, "ClawServo")
        );

        // Initialize Extendo Servos
        extendoServos = new ExtendoServos(
                hardwareMap.get(Servo.class, "ExtendoServoRight"),
                hardwareMap.get(Servo.class, "ExtendoServoLeft")
        );

        // Initialize Intake Servos
        intakeServos = new IntakeServos(
                hardwareMap.get(Servo.class, "IntakeServoRight"),
                hardwareMap.get(Servo.class, "IntakeServoLeft")
        );

    }


    @Override
    public void loop() {
        // Update follower and path state
        follower.update();
        autonomousPathUpdate();

        // Update subsystems
        viperSlides.update();

        // Telemetry feedback
        telemetry.addData("Slide Target", viperSlides.getTarget());
        telemetry.addData("Slide Position", viperSlides.getSlidePosition());
        telemetry.addData("Extendo State", extendoServos.isExtended() ? "Extended" : "Retracted");


        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }
}