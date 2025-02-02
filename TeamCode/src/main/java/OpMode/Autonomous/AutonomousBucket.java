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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name = "AutonomousBucket", group = "Autonomous")
public class AutonomousBucket extends OpMode {

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

    private final Pose startPose = new Pose(7, 104, Math.toRadians(270));
    private final Pose bucketPose = new Pose(18, 126, Math.toRadians(315));
    private final Pose blockPose1 = new Pose(27, 121, Math.toRadians(0));
    private final Pose blockPose2 = new Pose(27, 131, Math.toRadians(0));
    private final Pose blockpose3 = new Pose(45, 128, Math.toRadians(90));
    private final Pose endPose = new Pose(60, 96, Math.toRadians(90));
    private final Pose endPoseControlPoint = new Pose(60,124);



    //Control Points
    private final Pose controlPoint1 = new Pose(40, 72);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain pathChain1, pathChain2, pathChain3, pathChain4, pathChain5, pathChain6, pathChain7, pathChain8, pathChain9, pathChain10, pathChain11, pathChain12;


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        // Start pose to first score pose
        pathChain1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();
        // First score pose curve to first block
        pathChain2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose1)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();
        // Push first block to zone
        pathChain3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose1),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
        // Curve behind second block
        pathChain4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose2)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90))
                .build();
        // Push second block to zone
        pathChain5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose2),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Short path to get the clip on wall
        pathChain6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockpose3)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        // Wall to score pose 2
        pathChain7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockpose3),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        // Score pose 2 to wall
        pathChain8 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(endPose),
                        new Point(endPoseControlPoint)
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
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                follower.followPath(pathChain1,0.7 , true);
                pathTimer.resetTimer();
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position , then curve to first block

                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 3.5) {
                        bucketServoRight.setPosition(1);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 3.5 && pathTimer.getElapsedTimeSeconds() < 5) {
                        setPathState(2);
                        pathTimer.resetTimer();


                    }

                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
                    follower.followPath(pathChain2,0.6, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    linkageController.setPosition(LinkageController.Position.EXTENDED);


                }
                if (pathTimer.getElapsedTimeSeconds() > 2.6 && pathTimer.getElapsedTimeSeconds() < 2.7){
                    intakeServos.intakePosition();
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.8 && pathTimer.getElapsedTimeSeconds() < 6 ){
                    intakeMotor.intake();

                }

                if (pathTimer.getElapsedTimeSeconds() > 6.1 && pathTimer.getElapsedTimeSeconds() < 6.2 ){
                    linkageController.setPosition(LinkageController.Position.RETRACTED);
                    intakeServos.transferPosition();

                }

                if (pathTimer.getElapsedTimeSeconds() > 6.5 && pathTimer.getElapsedTimeSeconds() < 8 ){
                    intakeMotor.outtake();

                }


                  if (pathTimer.getElapsedTimeSeconds() > 8 && pathTimer.getElapsedTimeSeconds() < 8.2) {
                      setPathState(3);
                      pathTimer.resetTimer();

                }

                // setPathState(3);

                break;

            case 3: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain3,0.6, true);

                    if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
                        viperSlides.setTarget(ViperSlides.Target.HIGH);
                    }


                    if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.8){
                        bucketServoRight.setPosition(1);
                    }
                     if (pathTimer.getElapsedTimeSeconds() > 5.8 && pathTimer.getElapsedTimeSeconds() < 6){
                         setPathState(4);
                         pathTimer.resetTimer();
                    }

                }
                break;

              case 4: //
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 2){
                        bucketServoRight.setPosition(0);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.2){
                        viperSlides.setTarget(ViperSlides.Target.GROUND);
                    }

                    if (pathTimer.getElapsedTimeSeconds() >2.2 && pathTimer.getElapsedTimeSeconds() < 4) {
                        follower.followPath(pathChain4, 0.6, true);
                    }

                    if (pathTimer.getElapsedTimeSeconds() >4 && pathTimer.getElapsedTimeSeconds() < 5){
                        setPathState(5);
                        pathTimer.resetTimer();
                    }

                }
                break;

            case 5: //
                if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
                    follower.followPath(pathChain5,0.6, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    linkageController.setPosition(LinkageController.Position.EXTENDED);


                }
                if (pathTimer.getElapsedTimeSeconds() > 2.6 && pathTimer.getElapsedTimeSeconds() < 2.7){
                    intakeServos.intakePosition();
                }

                if (pathTimer.getElapsedTimeSeconds() > 2.8 && pathTimer.getElapsedTimeSeconds() < 6 ){
                    intakeMotor.intake();

                }

                if (pathTimer.getElapsedTimeSeconds() > 6.1 && pathTimer.getElapsedTimeSeconds() < 6.2 ){
                    linkageController.setPosition(LinkageController.Position.RETRACTED);
                    intakeServos.transferPosition();

                }

                if (pathTimer.getElapsedTimeSeconds() > 6.5 && pathTimer.getElapsedTimeSeconds() < 8 ){
                    intakeMotor.outtake();

                }


                if (pathTimer.getElapsedTimeSeconds() > 8 && pathTimer.getElapsedTimeSeconds() < 8.2) {
                    setPathState(6);
                    pathTimer.resetTimer();

                }
                break;

            case 6: //
                if (!follower.isBusy()) {
                    follower.followPath(pathChain6,0.6, true);

                    if (pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.6) {
                        viperSlides.setTarget(ViperSlides.Target.HIGH);
                    }


                    if (pathTimer.getElapsedTimeSeconds() > 5 && pathTimer.getElapsedTimeSeconds() < 5.8){
                        bucketServoRight.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 5.8 && pathTimer.getElapsedTimeSeconds() < 6){
                        setPathState(7);
                        pathTimer.resetTimer();
                    }

                }
                break;

            case 7: //
                if (!follower.isBusy()) {

                    if (pathTimer.getElapsedTimeSeconds() > 0 && pathTimer.getElapsedTimeSeconds() < 2){
                        bucketServoRight.setPosition(0);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 2.2){
                        viperSlides.setTarget(ViperSlides.Target.GROUND);
                    }

                    if (pathTimer.getElapsedTimeSeconds() >2.2 && pathTimer.getElapsedTimeSeconds() < 4) {
                        follower.followPath(pathChain7, 0.6, true);
                    }

                    if (pathTimer.getElapsedTimeSeconds() >4 && pathTimer.getElapsedTimeSeconds() < 5){
                        setPathState(8);
                        pathTimer.resetTimer();
                    }

                }
                break;

            case 8: if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 8) {

                follower.followPath(pathChain7, 0.6, true);
                pathTimer.resetTimer();
                setPathState(9);
            }

                break;

            case 9: //
                if (!follower.isBusy()) {

                    viperSlides.setTarget(ViperSlides.Target.LOW);

                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 4) {
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 4) {
                        setPathState(10);


                    }

                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 2.2 && pathTimer.getElapsedTimeSeconds() < 3) {
                    follower.followPath(pathChain8,0.85, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);


                }


                if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 4) {
                    pathTimer.resetTimer();
                    setPathState(-1);

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

        viperSlides.resetPosition();




        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));
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

        linkageController.resetEncoder();

        linkageController.setPosition(LinkageController.Position.RETRACTED);

        /* linkageController.zeroMotor();

        while (!linkageController.isAtTarget()) {
            linkageController.checkForAmperageSpike();
            telemetry.addData("Zeroing...", "Current Position: %d", linkageController.getCurrentPosition());
            telemetry.update();
        } */




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
        // linkageController.setPosition(LinkageController.Position.RETRACTED);
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
        linkageController.setPosition(LinkageController.Position.RETRACTED);
        //linkageController.checkForAmperageSpike();
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