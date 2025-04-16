package OpMode.Autonomous;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
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

@Autonomous(name = "AutonomousOliver", group = "Autonomous")
public class AutonomousOliverTest extends OpMode {

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
    private IntakeServos intakeServos; // IntakeBoolean subsystem instance
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

    private final Pose startPose = new Pose(8, 56, Math.toRadians(180));





    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;



        public static PathBuilder builder = new PathBuilder();

        public static PathChain line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.000, 56.000, Point.CARTESIAN),
                                new Point(38.000, 77.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line2 = builder
                .addPath(
                        new BezierCurve(
                                new Point(38.000, 77.000, Point.CARTESIAN),
                                new Point(17.000, 13.000, Point.CARTESIAN),
                                new Point(63.000, 50.000, Point.CARTESIAN),
                                new Point(58.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line3 = builder
                .addPath(
                        new BezierLine(
                                new Point(58.000, 25.000, Point.CARTESIAN),
                                new Point(20.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line4 = builder
                .addPath(
                        new BezierCurve(
                                new Point(20.000, 25.000, Point.CARTESIAN),
                                new Point(66.000, 28.000, Point.CARTESIAN),
                                new Point(58.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line5 = builder
                .addPath(
                        new BezierLine(
                                new Point(58.000, 16.000, Point.CARTESIAN),
                                new Point(20.000, 16.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        public static PathChain line6 = builder
                .addPath(
                        new BezierLine(
                                new Point(20.000, 16.000, Point.CARTESIAN),
                                new Point(14.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        public static PathChain line7 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.000, 34.000, Point.CARTESIAN),
                                new Point(8.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line8 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 74.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        public static PathChain line9 = builder
                .addPath(
                        new BezierLine(
                                new Point(38.000, 74.000, Point.CARTESIAN),
                                new Point(14.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        public static PathChain line10 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.000, 34.000, Point.CARTESIAN),
                                new Point(8.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line11 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 71.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        public static PathChain line12 = builder
                .addPath(
                        new BezierLine(
                                new Point(38.000, 71.000, Point.CARTESIAN),
                                new Point(14.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        public static PathChain line13 = builder
                .addPath(
                        new BezierLine(
                                new Point(14.000, 34.000, Point.CARTESIAN),
                                new Point(8.000, 34.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        public static PathChain line14 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.000, 34.000, Point.CARTESIAN),
                                new Point(38.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        public static PathChain line15 = builder
                .addPath(
                        new BezierLine(
                                new Point(38.000, 68.000, Point.CARTESIAN),
                                new Point(10.226, 15.590, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();



    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                setPathState(1);
                break;
            case 1:
                if(!follower.atParametricEnd()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(2);
                }
            case 2:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    clawServo.openPosition();
                    setPathState(3);
                }
            case 3:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line2,false);
                    setPathState(4);
                }
            case 4:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(5);
                }
            case 5:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line3,false);
                    setPathState(6);
                }
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(line4,false);
                    setPathState(7);
                }
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(line5,false);
                    setPathState(8);
                }
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(line6);
                    setPathState(9);
                }
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(line7);
                    setPathState(10);
                }
            case 10:
                if(!follower.isBusy()) {
                    clawServo.closedPosition();
                    setPathState(11);
                }
            case 11:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line8);
                    setPathState(12);
                }
            case 12:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(13);
                }
            case 13:
                if(!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(14);
                }
            case 14:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    clawServo.openPosition();
                    setPathState(15);
                }
            case 15:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line9);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(16);
                }
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(line10);
                    setPathState(17);
                }
            case 17:
                if(!follower.isBusy()) {
                    clawServo.closedPosition();
                    setPathState(18);
                }
            case 18:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line11);
                    setPathState(19);
                }
            case 19:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(20);
                }
            case 20:
                if(!follower.isBusy()) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(21);
                }
            case 21:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    clawServo.openPosition();
                    setPathState(22);
                }
            case 22:
                if(pathTimer.getElapsedTimeSeconds() >0.5) {
                    follower.followPath(line12);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(-1);
                }






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
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeServos.transferPosition(); // Set intake servos to transfer position

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServos.transferPosition();
        clawServo.closedPosition();



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
    }



    @Override
    public void stop() {
    }
}