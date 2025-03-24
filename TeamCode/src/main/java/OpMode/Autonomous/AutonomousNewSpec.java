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
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServosNEW;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutonomousNewSpec", group = "Autonomous")
public class AutonomousNewSpec extends OpMode{


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

    //Pose

    private final Pose startPose = new Pose(6.5,56,180);

    private final Pose rackPose = new Pose(33,70,180);

    private final Pose wallPose = new Pose(11,26,0);

    public static Pose finalPose =new Pose();

    //control point




    /* Paths and PathChains */
    private Path a ;

    private PathChain startPath, toRack, toWall;




    public void buildPaths() {

        startPath =follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(rackPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(),rackPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();



        toRack = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(wallPose),
                        new Point(rackPose)
                ))
                .setLinearHeadingInterpolation(wallPose.getHeading(),rackPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        toWall = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(rackPose),
                        new Point(wallPose)
                ))
                .setLinearHeadingInterpolation(rackPose.getHeading(),wallPose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();


    }


    //case

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(startPath,0.8,true);
                setPathState(1);
                break;


            case 1:
                viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                setPathState(2);
                break;


            case 2:
                if (!follower.isBusy()){
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    clawServo.openPosition();
                    setPathState(3);

                }

                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds()>0.75){
                    follower.followPath(toWall,0.8,true);
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()){
                    clawServo.closedPosition();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds()>0.5){
                    follower.followPath(toRack,0.8,true);
                    viperSlides.setTarget(ViperSlides.Target.MEDIUM);
                    setPathState(6);
                }
                break;


            case 6:
                if (!follower.isBusy()){
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    clawServo.openPosition();
                    setPathState(-1);
                }
                break;

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

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServos.transferPosition();
        clawServo.closedPosition();

        linkageController.zeroMotor();


    }



    @Override
    public void init_loop() {
        follower.update();
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();

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