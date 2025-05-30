package Unused;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColorAndDistance;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Deprecated
@Autonomous(name = "AutonomousBucket", group = "Autonomous")
public class AutonomousBucket extends OpMode {

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;

    private Follower follower;
    private Telemetry telemetryA;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServos intakeServos; // IntakeBoolean subsystem instance
    private ClawServo clawServo;


    private Servo bucketServoRight;
    private ColorSensor colorSensor;
    private ColorAndDistance colorAndDistance;

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

    private final Pose startPose = new Pose(7, 104, Math.toRadians(270));
    private final Pose bucketPose = new Pose(18, 126, Math.toRadians(315));
    private final Pose blockPose1 = new Pose(25, 121, Math.toRadians(0));
    private final Pose blockPose2 = new Pose(25, 131, Math.toRadians(0));
    private final Pose blockPose3 = new Pose(35, 126, Math.toRadians(57));
    private final Pose endPose = new Pose(60, 96, Math.toRadians(90));



    //controle point
    private final Pose endPoseControlPoint = new Pose(60,124);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain startPath, blockPath1, bucketPath1, blockPath2, bucketPath2, blockPath3, bucketPath3, endPath;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        // Start pose to first score pose
        startPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(315))
                .build();
        // First score pose curve to first block
        blockPath1= follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose1)
                ))
                .setConstantHeadingInterpolation( Math.toRadians(0))
                .build();
        // Push first block to zone
        bucketPath1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose1),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
        // Curve behind second block
        blockPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose2)
                ))
                .setConstantHeadingInterpolation( Math.toRadians(0))
                .build();
        // Push second block to zone
        bucketPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose2),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                .build();
        // Short path to get the clip on wall
        blockPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose3)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(57))
                .build();
        // Wall to score pose 2
        bucketPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose3),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(57), Math.toRadians(315))
                .build();
        // Score pose 2 to wall
        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(endPoseControlPoint),
                        new Point(endPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(90))
                .build();








    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Do actions , then Move from start to scoring position 1
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                follower.followPath(startPath, 0.7, true);
                pathTimer.resetTimer();
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position , then curve to first block

                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 2) {
                        bucketServos.depositPosition();

                    }


                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3.6) {
                        linkageController.setPosition(LinkageController.Position.EXTENDED);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 4.5) {
                        bucketServos.transferPosition();
                    }


                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 5.5) {
                        follower.followPath(blockPath1);
                        setPathState(2);
                        pathTimer.resetTimer();

                    }

                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 2) {
                    intakeMotor.intake();
                }
                if (pathTimer.getElapsedTimeSeconds()>0 && pathTimer.getElapsedTimeSeconds()<0.1){
                    intakeServos.intakePosition();
                }

                if (pathTimer.getElapsedTimeSeconds()>2 && pathTimer.getElapsedTimeSeconds() <2.1) {
                    intakeMotor.stop();
                    setPathState(3);
                    pathTimer.resetTimer();
                }
                break;

            case 3:

                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    pathTimer.resetTimer();
                    setPathState(4);
                }

                break;


            case 4:
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        intakeServos.transferPosition();

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.3) {
                        linkageController.setPosition(LinkageController.Position.RETRACTED);

                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.4 && pathTimer.getElapsedTimeSeconds() < 1.7) {
                        intakeMotor.intake();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.7 && pathTimer.getElapsedTimeSeconds() < 1.7) {
                        intakeMotor.stop();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.7 && pathTimer.getElapsedTimeSeconds() < 1.8) {
                        pathTimer.resetTimer();
                        setPathState(5);

                    }
                }

                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(bucketPath1, 0.6, true);
                    viperSlides.setTarget(ViperSlides.Target.HIGH);
                    pathTimer.resetTimer();
                    setPathState(6);
                }

                break;


            case 6: //
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        bucketServos.depositPosition();


                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.2) {
                        linkageController.setPosition(LinkageController.Position.EXTENDED);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 0.3) {
                        bucketServos.transferPosition();
                    }


                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 0.6) {
                        setPathState(7);
                        pathTimer.resetTimer();

                    }

                }
                break;

            case 7: //
                if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(blockPath2);
                }
                if (pathTimer.getElapsedTimeSeconds()>0.1 && pathTimer.getElapsedTimeSeconds()<2){
                    intakeMotor.intake();
                }
                if (pathTimer.getElapsedTimeSeconds()>0.1 && pathTimer.getElapsedTimeSeconds()<0.2){
                    intakeServos.intakePosition();
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.1) {
                    intakeMotor.stop();
                    setPathState(8);
                    pathTimer.resetTimer();
                }
                break;
            case 8:

                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    pathTimer.resetTimer();
                    setPathState(9);
                }
                break;

            case 9: //
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        linkageController.setPosition(LinkageController.Position.RETRACTED);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 0.6) {
                        intakeServos.transferPosition();

                    }


                    if (pathTimer.getElapsedTimeSeconds() > 0.4 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        intakeMotor.intake();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.5 && pathTimer.getElapsedTimeSeconds() < 1.6) {
                        intakeMotor.stop();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.6 && pathTimer.getElapsedTimeSeconds() < 1.7) {
                        pathTimer.resetTimer();
                        setPathState(10);

                    }
                }

                break;

            case 10: //
                if (!follower.isBusy()) {
                    follower.followPath(bucketPath2, 0.6, true);
                    viperSlides.setTarget(ViperSlides.Target.HIGH);
                    pathTimer.resetTimer();
                    setPathState(12);
                }
                break;

                case 12:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        bucketServos.depositPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.1 && pathTimer.getElapsedTimeSeconds() < 0.2) {
                        linkageController.setPosition(LinkageController.Position.EXTENDED);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5 && pathTimer.getElapsedTimeSeconds() < 0.6) {
                        pathTimer.resetTimer();
                        setPathState(13);

                    }
                }
                break;

            case 13: //
                if (!follower.isBusy()) {
                    follower.followPath(blockPath3, 0.6, true);
                    bucketServos.transferPosition();

                    if (pathTimer.getElapsedTimeSeconds()>1 && pathTimer.getElapsedTimeSeconds()<1.1){
                        intakeServos.intakePosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds()>1.1 && pathTimer.getElapsedTimeSeconds()<2){
                        intakeMotor.intake();
                    }

                    if(pathTimer.getElapsedTimeSeconds()>2 && pathTimer.getElapsedTimeSeconds()<2.1) {
                        pathTimer.resetTimer();
                        setPathState(14);
                    }
                }


                break;
            case 14:

                if (!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    pathTimer.resetTimer();
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 0.1) {
                        intakeServos.transferPosition();

                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.3) {
                        linkageController.setPosition(LinkageController.Position.RETRACTED);

                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1.4 && pathTimer.getElapsedTimeSeconds() < 3) {
                        intakeMotor.outtake();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 2.4 && pathTimer.getElapsedTimeSeconds() < 4) {
                        intakeMotor.stop();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 2.4 && pathTimer.getElapsedTimeSeconds() < 5) {
                        pathTimer.resetTimer();
                        setPathState(16);

                    }
                }

                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(bucketPath3, 0.6, true);
                    viperSlides.setTarget(ViperSlides.Target.HIGH);
                    pathTimer.resetTimer();
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                        bucketServos.depositPosition();
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 6) {
                        pathTimer.resetTimer();
                        setPathState(18);
                    }

                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 4) {
                        bucketServos.transferPosition();

                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 4) {
                        follower.followPath(endPath, 0.8, true);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 1.1) {
                        viperSlides.setTarget(ViperSlides.Target.LOW);
                    }




                    if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 8) {
                        pathTimer.resetTimer();
                        setPathState(19);

                    }
                }

                break;

            case 19 :

                viperSlides.setTarget(ViperSlides.Target.LOW);
                setPathState(-1);



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




        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));
        colorAndDistance = new ColorAndDistance(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));
        intakeServos.transferPosition(); // Set intake servos to transfer position
        //Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServos.transferPosition();
        clawServo.openPosition();

        linkageController.zeroMotor();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());






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
        follower.telemetryDebug(telemetryA);

        telemetryA.update();



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