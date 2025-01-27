package OpMode.Autonomous;


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
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutonomousSpec", group = "Autonomous")
public class AutonomousSpec extends OpMode {

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;

    private Follower follower;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServos intakeServos; // Intake subsystem instance
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // Intake Motor and Color Sensor
    private DcMotor intakemotor;
    private IntakeMotor intakeMotor;

    // Loop Timer
    private ElapsedTime loopTimer;

    // Declare the LinkageController instance
    private LinkageController linkageController;



    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    private final Pose startPose = new Pose(8, 56, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(8, 80, Math.toRadians(270));
    private final Pose blockPose1 = new Pose(8, 80, Math.toRadians(270));
    private final Pose blockPush1 = new Pose(8, 80, Math.toRadians(270));
    private final Pose blockPose2 = new Pose(8, 80, Math.toRadians(270));
    private final Pose blockPush2 = new Pose(8, 80, Math.toRadians(270));
    private final Pose wallPose = new Pose(8, 80, Math.toRadians(270));
    private final Pose scorePose2 = new Pose(8, 80, Math.toRadians(270));
    private final Pose scorePose3 = new Pose(8, 80, Math.toRadians(270));
    private final Pose scorePose4 = new Pose(8, 80, Math.toRadians(270));
    private final Pose parkPose = new Pose(8, 80, Math.toRadians(270));



    //Control Points
    private final Pose parkControlPose = new Pose(64, 134);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain pathChain1, pathChain2, pathChain3, pathChain4, pathChain5, pathChain6, pathChain7, pathChain8, pathChain9, pathChain10, pathChain11, pathChain12;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        // Start pose to first score pose
        pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(8.000, 56.000),
                        new Point(30, 74)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // First score pose curve to first block
        pathChain2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(30, 74, Point.CARTESIAN),
                        new Point(18.368, 14.903, Point.CARTESIAN),
                        new Point(70.700, 48.347, Point.CARTESIAN),
                        new Point(59, 25.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // Push first block to zone
        pathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(58.917, 25.000),
                        new Point(14.000, 25.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // Curve behind second block
        pathChain4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(14.000, 25.000),
                        new Point(61.690, 33.617),
                        new Point(60.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Push second block to zone
        pathChain5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(60.000, 15.000),
                        new Point(14.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Short path to get the clip on wall
        pathChain6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(14.000, 15.000),
                        new Point(11, 24)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Wall to score pose 2
        pathChain7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(11, 24),
                        new Point(30, 68.621)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 2 to wall
        pathChain8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(30, 68.621),
                        new Point(11, 24)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to score pose 3
        pathChain9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(11, 24),
                        new Point(30, 64.635)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 3 to wall
        pathChain10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(30, 64.635),
                        new Point(11, 24)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to Score pose 4
        pathChain11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(11, 24),
                        new Point(30, 61.170)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 4 to park
        pathChain12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(30, 61.170),
                        new Point(10, 21)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();








    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Do actions , then Move from start to scoring position 1
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                follower.followPath(pathChain1, true);
                pathTimer.resetTimer();
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position , then curve to first block

                if (!follower.isBusy()) {

                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 4) {
                        clawServo.openPosition();
                        viperSlides.setTarget(ViperSlides.Target.GROUND);
                    }
                     if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 5) {
                        follower.followPath(pathChain2, true);


                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5.1 && pathTimer.getElapsedTimeSeconds() < 6) {
                        pathTimer.resetTimer();
                        setPathState(2);

                    }

                }
                break;

            case 2: //

                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 3) {

                        follower.followPath(pathChain3, true);

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 3.1 && pathTimer.getElapsedTimeSeconds() < 6) {


                        setPathState(3);
                        pathTimer.resetTimer();

                    }


                }
                break;

            case 3: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 1) {

                        follower.followPath(pathChain4, true);

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 2) {


                        setPathState(4);
                        pathTimer.resetTimer();

                    }

                    //follower.followPath(pathChain4, true);
                    //pathTimer.resetTimer();
                    //setPathState(4);
                }
                break;

            case 4: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain5, true);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;

            case 5: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain6, true);
                    pathTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.5) {
                        clawServo.closedPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    }

                    follower.followPath(pathChain7, true);
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;

            case 7: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.LOW);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && pathTimer.getElapsedTimeSeconds() < 2) {
                        clawServo.openPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        follower.followPath(pathChain8, true);
                        viperSlides.setTarget(ViperSlides.Target.GROUND);

                    }
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;

            case 8: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.5) {
                        clawServo.closedPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    }
                    follower.followPath(pathChain9, true);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.LOW);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && pathTimer.getElapsedTimeSeconds() < 2) {
                        clawServo.openPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        follower.followPath(pathChain10, true);
                        viperSlides.setTarget(ViperSlides.Target.GROUND);

                    }
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;

            case 10: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.5) {
                        clawServo.closedPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    }
                    follower.followPath(pathChain11, true);
                    pathTimer.resetTimer();
                    setPathState(11);
                }
                break;

            case 11: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        viperSlides.setTarget(ViperSlides.Target.LOW);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && pathTimer.getElapsedTimeSeconds() < 2) {
                        clawServo.openPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        follower.followPath(pathChain12, true);
                        viperSlides.setTarget(ViperSlides.Target.GROUND);

                    }
                    pathTimer.resetTimer();
                    setPathState(-1); // End the autonomous routine
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/



    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        viperSlides = new ViperSlides(
                hardwareMap.get(DcMotorEx.class, "slidemotorleft"),
                hardwareMap.get(DcMotorEx.class, "slidemotorright"),
                hardwareMap.get(TouchSensor.class, "limitSwitch"),
                p, i, d
        );

        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeServos.transferPosition(); // Set intake servos to transfer position
        //Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServos.transferPosition();
        clawServo.closedPosition();



        linkageController.zeroMotor();
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        while (linkageController.isZeroing) {
            linkageController.checkForAmperageSpike();
            telemetry.addData("Zeroing...", "Current Position: %d", linkageController.getCurrentPosition());
            telemetry.update();
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
        viperSlides.update();

        /* if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            // Limit switch is pressed, and it wasn't pressed in the previous loop iteration
            viperSlides.resetPosition();
        } */

        // Update the previous state of the limit switch
        wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}