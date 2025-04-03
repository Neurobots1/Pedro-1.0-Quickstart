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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "AutonomousCLIP", group = "Autonomous")
public class AutonomousCLIP extends OpMode {

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
    private HandServo handServo;



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

    private final Pose startPose = new Pose(9, 57, Math.toRadians(180));
    private final Pose PrehumanPose = new Pose(20, 30, Math.toRadians(0));
    private final Pose HumanPose = new Pose(7.5, 27, Math.toRadians(0));
    private final Pose ClipPose1 = new Pose(35.5, 75, Math.toRadians(180));
    private final Pose BlocPose1 = new Pose(55, 33, Math.toRadians(180));
    private final Pose BlocFin1 = new Pose(20, 14, Math.toRadians(180));
    private final Pose BlocPose2 = new Pose(66, 18, Math.toRadians(0));
    private final Pose BlocPose3 = new Pose(66, 9, Math.toRadians(0));
    private final Pose ClipPose2 = new Pose(38, 73, Math.toRadians(180));
    private final Pose ClipPose3 = new Pose(38, 70, Math.toRadians(180));
    private final Pose ClipPose4 = new Pose(60, 97, Math.toRadians(180));
    private final Pose ClipPose5 = new Pose(75, 97,Math.toRadians(180));
    private final Pose endPose = new Pose(60, 91.5, Math.toRadians(-90));

    public static Pose finalPose =new Pose();








    /* Paths and PathChains */
    private Path scorePreload, park;
    private PathChain startPath, FirstClip , ClipPath2 , ClipPath3 , ClipPath4 , ClipPath5 , BlockPath1 , BlockFin1 , BlockHumain , BlockPath2, BlockMid2 ,BlockFin2 , BlockPath3 , ClipPath9 , PrehumanPath;


    public void buildPaths() {

        startPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(ClipPose1)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.5)
                .build();

        FirstClip = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(ClipPose1),
                        new Point(PrehumanPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1)
                .build();


        PrehumanPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(PrehumanPose),
                        new Point(HumanPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        ClipPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(HumanPose),
                        new Point(ClipPose2)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(176))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        ClipPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(ClipPose2),
                        new Point(PrehumanPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        ClipPath4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(PrehumanPose),
                        new Point(6.5, 30)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(-10))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        ClipPath5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(6.5, 28),
                        new Point(ClipPose3)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-10),Math.toRadians(176))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        BlockPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(ClipPose1),
                        new Point(1, 39),
                        new Point(BlocPose1)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        BlockFin1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(BlocPose1),
                        new Point(BlocFin1)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        BlockHumain = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(BlocFin1),
                        new Point(PrehumanPose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        BlockPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(BlocFin1),
                        new Point(BlocPose1)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1)
                .build();

        BlockMid2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(BlocPose1),
                        new Point(BlocPose2)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.5)
                .build();

        BlockFin2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(BlocPose2),
                        new Point(BlocFin1)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180),Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.5)
                .build();

        BlockPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(6.5, 28),
                        new Point(ClipPose3)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-10),Math.toRadians(176))
                .setZeroPowerAccelerationMultiplier(1)
                .build();




        /** ---------------- ALTERNATIVE PATHS ---------------- **/


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:  // Move to scoring position 1
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                follower.followPath(startPath, 0.7, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()){
                viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds()> 0.3){
                 clawServo.openPosition();
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds()> 0.8){
                    follower.followPath(BlockPath1, 0.8, true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()){
                    follower.followPath(BlockFin1, 0.8, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()){
                    follower.followPath(BlockPath2, 0.8, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()){
                    follower.followPath(BlockMid2,0.8,true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()){
                    follower.followPath(BlockFin2,0.8,true);
                    setPathState(8);
                }
                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds()>0.5){
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(9);
                }
                break;


            case 9:
                if (pathTimer.getElapsedTimeSeconds()> 0.3){
                    follower.followPath(ClipPath2, 0.8, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds()>0.6){
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()){
                    clawServo.openPosition();
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds()>0.5){
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    follower.followPath(ClipPath5,0.8,true);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()){
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(14);
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds()>0.5){
                    clawServo.openPosition();
                    setPathState(-1);
                }
                break;




            /** ---------------- ALTERNATIVE PATHS ---------------- **/


        }



    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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
        handServo = new HandServo(hardwareMap.get(Servo.class, "HandServo"));
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServosNEW(intakeServoRight , intakeServoLeft);
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));
        colorAndDistance = new ColorAndDistance(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));
        intakeServos.transferPosition(); // Set intake servos to transfer position
        //Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);

        handServo.closedPosition();

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));


        bucketServos.transferPosition();
        clawServo.closedPosition();

        linkageController.zeroMotor();






    }



    @Override
    public void init_loop() {

    }


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        viperSlides.setPIDEnabled(true);
        setPathState(0);
    }


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



    @Override
    public void stop() {
        finalPose= new Pose(follower.getPose().getX(),follower.getPose().getY(),follower.getPose().getHeading()).copy();
    }
}