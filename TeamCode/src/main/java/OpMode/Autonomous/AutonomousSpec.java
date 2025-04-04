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
import OpMode.Subsystems.IntakeServosNEW;
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
    private IntakeServosNEW intakeServos; // IntakeBoolean subsystem instance
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // IntakeBoolean Motor and Color Sensor
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
    private final Pose scorePose = new Pose(35, 76, Math.toRadians(180));
    private final Pose wallPose = new Pose(8, 20, Math.toRadians(0));


    //Control Points
    private final Pose parkControlPose = new Pose(64, 134);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain startPath, ToBlockPath1, blockPath1, ToBlockPath2, blockPath2, WallPath1, scorePath1, WallPath2, scorePath2, pathChain10, pathChain11, pathChain12;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        // Start pose to first score pose
        startPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(scorePose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        // First score pose curve to first block
        ToBlockPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(15, 10),
                        new Point(70.700, 48.347),
                        new Point(59, 25.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // Push first block to zone
        blockPath1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(59, 25.000),
                        new Point(18, 25.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        // Curve behind second block
        ToBlockPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(18, 25.000),
                        new Point(61.690, 33.617),
                        new Point(60.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Push second block to zone
        blockPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(60.000, 15.000),
                        new Point(18.000, 15.000)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Short path to get the clip on wall
        WallPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(18.000, 15.000),
                        new Point(32,21),
                        new Point(wallPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Wall to score pose 2
        scorePath1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(wallPose),
                        new Point(scorePose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 2 to wall
        WallPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(21.000, 44.000, Point.CARTESIAN),
                        new Point(42.500, 24.000, Point.CARTESIAN),
                        new Point(wallPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to score pose 3
        scorePath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(wallPose),
                        new Point(scorePose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 3 to wall
        pathChain10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(wallPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        // Wall to Score pose 4
        pathChain11 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(wallPose),
                        new Point(scorePose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 4 to park
        pathChain12 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(12, 20)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();








    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Do actions , then Move from start to scoring position 1
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                follower.followPath(startPath,0.7 , true);
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                pathTimer.resetTimer();
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position , then curve to first block

                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    if (pathTimer.getElapsedTimeSeconds() > 2.9 && pathTimer.getElapsedTimeSeconds() < 3) {
                        clawServo.openPosition();
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds()>0.3) {

                    follower.followPath(ToBlockPath1, 0.9, false);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(3);
                }
                break;

            case 3: //
                if (!follower.isBusy()) {
                    follower.followPath(blockPath1,0.6, false);
                    setPathState(4);

                }
                break;

              case 4: //
                if (!follower.isBusy()) {
                    follower.followPath(ToBlockPath2,0.9, false);
                    setPathState(5);

                }
                break;

            case 5: //
                if (!follower.isBusy()) {
                    follower.followPath(blockPath2, 0.6,false);
                    setPathState(6);
                }
                break;

            case 6: //
                if (!follower.isBusy()) {
                    follower.followPath(WallPath1,0.5, true);
                    setPathState(7);
                }
                break;

            case 7: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
                        clawServo.closedPosition();
                        setPathState(21);
                    }
                }
                break;

            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 0.6) {
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(8);
                }

            case 8:
                follower.followPath(scorePath1, 0.85, true);
                pathTimer.resetTimer();
                setPathState(9);
                break;

            case 9: //
                if (!follower.isBusy()) {
                        viperSlides.setTarget(ViperSlides.Target.LOW);
                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 3.1) {
                        clawServo.openPosition();
                        setPathState(10);
                    }
                }
                break;

            case 10:
                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        follower.followPath(WallPath2, 0.9, true);
                        viperSlides.setTarget(ViperSlides.Target.GROUND);
                        setPathState(11);
                    }
                break;

            case 11: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.1) {
                        clawServo.closedPosition();
                        setPathState(22);
                    }
                }
                break;

            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.1) {
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(12);
                }
                break;

            case 12:

                follower.followPath(scorePath2, 0.7, true);
                pathTimer.resetTimer();
                setPathState(13);
                break;

            case 13: //
                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    if (pathTimer.getElapsedTimeSeconds() > 3.7 && pathTimer.getElapsedTimeSeconds() < 4) {
                        clawServo.openPosition();
                        setPathState(14);
                    }
                }
                break;

            case 14:

                if(!follower.isBusy()) {

                        follower.followPath(pathChain10, 0.9, true);
                        viperSlides.setTarget(ViperSlides.Target.GROUND);
                        pathTimer.resetTimer();
                        setPathState(-1);

                }



                break;

             /* case 15: //
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 3.7 && pathTimer.getElapsedTimeSeconds() < 4) {
                        clawServo.closedPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 4.2) {
                        viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                        setPathState(16);
                    }





                }
                break;

            case 16:

                follower.followPath(pathChain11, 0.85, true);
                pathTimer.resetTimer();
                setPathState(17);


                break;

            case 17: //
                if (!follower.isBusy()) {

                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    if (pathTimer.getElapsedTimeSeconds() > 3.7 && pathTimer.getElapsedTimeSeconds() < 4) {
                        clawServo.openPosition();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 3.7 && pathTimer.getElapsedTimeSeconds() < 4) {
                        setPathState(18);


                    }

                }
                break;

            case 18:

                    follower.followPath(pathChain12,0.85, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    pathTimer.resetTimer();
                    setPathState(-1);



                break; */


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

        viperSlides.resetPosition();

        //Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        linkageController.zeroMotor();




        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServosNEW(intakeServoRight , intakeServoLeft);
        intakeServos.transferPosition(); // Set intake servos to transfer position

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServos.transferPosition();
        clawServo.closedPosition();



    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        viperSlides.setPIDEnabled(true);
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

        linkageController.checkForAmperageSpike();
        linkageController.update();



        /* if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            // Limit switch is pressed, and it wasn't pressed in the previous loop iteration
            viperSlides.resetPosition();
        } */

        // Update the previous state of the limit switch
         // wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();
    }


    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}