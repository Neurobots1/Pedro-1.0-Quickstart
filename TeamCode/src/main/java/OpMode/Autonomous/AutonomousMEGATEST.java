package OpMode.Autonomous;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.GamePieceDetection;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutonomousMEGATEST", group = "Autonomous")
public class AutonomousMEGATEST extends OpMode {

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

    private final Pose startPose = new Pose(8, 80, Math.toRadians(270));
    private final Pose clipPose = new Pose(15, 125, Math.toRadians(315));
    private final Pose parkPose = new Pose(62, 96, Math.toRadians(90));
    private final Pose pickup1Pose = new Pose(32, 121, Math.toRadians(0));


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
                        new Point(37.949, 73.300)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // First score pose curve to first block
        pathChain2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(37.949, 73.300),
                        new Point(18.368, 14.903),
                        new Point(70.700, 48.347),
                        new Point(58.917, 25.000)
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
                        new Point(10.917, 24.606)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Wall to score pose 2
        pathChain7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(10.917, 24.606),
                        new Point(37.603, 68.621)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 2 to wall
        pathChain8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(37.603, 68.621),
                        new Point(11.090, 24.780)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to score pose 3
        pathChain9 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(11.090, 24.780),
                        new Point(37.256, 64.635)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 3 to wall
        pathChain10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(37.256, 64.635),
                        new Point(10.917, 24.260)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to Score pose 4
        pathChain11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(10.917, 24.260),
                        new Point(37.430, 61.170)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 4 to park
        pathChain12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(37.430, 61.170),
                        new Point(10.051, 20.968)
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
                //Actions
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                //
                follower.followPath(pathChain1, true);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position , then curve to first block
                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    //Wait for slides to go to low position
                    clawServo.openPosition();
                    // wait for claw to open
                    follower.followPath(pathChain2, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(2);
                }
                break;

            case 2: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain3, true);
                    setPathState(3);
                }
                break;

            case 3: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain4, true);
                    setPathState(4);
                }
                break;

            case 4: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain5, true);
                    setPathState(5);
                }
                break;

            case 5: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain6, true);
                    setPathState(6);
                }
                break;

            case 6: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain7, true);
                    setPathState(7);
                }
                break;

            case 7: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain8, true);
                    setPathState(8);
                }
                break;

            case 8: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain9, true);
                    setPathState(9);
                }
                break;

            case 9: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain10, true);
                    setPathState(10);
                }
                break;

            case 10: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain11, true);
                    setPathState(11);
                }
                break;

            case 11: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain12, true);
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
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
        viperSlides.update();

        if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            // Limit switch is pressed, and it wasn't pressed in the previous loop iteration
            viperSlides.resetPosition();
        }

        // Update the previous state of the limit switch
        wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}