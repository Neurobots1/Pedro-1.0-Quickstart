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

@Autonomous(name = "AutonomousNewBucket", group = "Autonomous")
public class AutonomousFSM extends OpMode {

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

    private final Pose startPose = new Pose(7, 104, Math.toRadians(270));
    private final Pose bucketPose = new Pose(14, 130, Math.toRadians(315));
    private final Pose blockPose1 = new Pose(20, 121, Math.toRadians(0));
    private final Pose blockPose2 = new Pose(20, 131, Math.toRadians(0));
    private final Pose blockPose3 = new Pose(33, 121, Math.toRadians(57));
    private final Pose endPose = new Pose(60, 96, Math.toRadians(90));



    //control point
    private final Pose endPoseControlPoint = new Pose(60,124);

    /* Paths and PathChains */
    private Path scorePreload, park;
    private PathChain startPath, blockPath1,  bucketPath1, blockPath2, blockPath2Alternative, bucketPath2, blockPath3, blockPath3Alternative,bucketPath3, endPath , endPathAlternative;


    public void buildPaths() {

        startPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(),bucketPose.getHeading())
                .build();

        blockPath1= follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose1)
                ))
                .setLinearHeadingInterpolation(bucketPose.getHeading(),blockPose1.getHeading())
                .build();


        bucketPath1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose1),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(blockPose1.getHeading(), bucketPose.getHeading())
                .build();

        blockPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose2)
                ))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), blockPose2.getHeading())
                .build();



        bucketPath2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose2),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(blockPose2.getHeading(), bucketPose.getHeading())
                .build();

        blockPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(bucketPose),
                        new Point(blockPose3)
                ))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), blockPose3.getHeading())
                .build();




        bucketPath3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockPose3),
                        new Point(bucketPose)
                ))
                .setLinearHeadingInterpolation(blockPose3.getHeading(), bucketPose.getHeading())
                .build();

        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(bucketPose),
                        new Point(endPoseControlPoint),
                        new Point(endPose)
                ))
                .setLinearHeadingInterpolation(bucketPose.getHeading(), endPose.getHeading())
                .build();


        /** ---------------- ALTERNATIVE PATHS ---------------- **/

        blockPath2Alternative = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(blockPose2)
                ))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),blockPose2.getHeading())
                .build();

        blockPath3Alternative = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(blockPose3)
                ))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),blockPose3.getHeading())
                .build();

        endPathAlternative = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(follower.getPose()),
                        new Point(endPose)
                ))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),blockPose3.getHeading())
                .build();








    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:  // Move to scoring position 1
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                follower.followPath(startPath, 0.7, true);
                setPathState(1);
                break;

            case 1:  // Deposit bucket
                if (!follower.isBusy()) {
                    bucketServos.depositPosition();
                    setPathState(2);
                }
                break;

            case 2:  // Transfer position & extend linkage for intake
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    bucketServos.transferPosition();
                    follower.followPath(blockPath1);
                    linkageController.setPosition(LinkageController.Position.EXTENDED);
                    setPathState(3);
                }
                break;

            case 3:  // Wait 2s, then lower slides
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    intakeMotor.intake();
                    setPathState(4);
                }
                break;

            case 4:  // Start intake
                if (!follower.isBusy()) {
                    intakeServos.intakePosition();
                    setPathState(5);
                }
                break;

            case 5:  // Wait for block detection
                colorAndDistance.update();
                if (colorAndDistance.getDetectedColor().equals("Yellow")) {
                    intakeMotor.stop();
                    setPathState(6);
                } else if (pathTimer.getElapsedTimeSeconds() > 3.7 && colorAndDistance.getDetectedColor().equals("None")) {
                    intakeMotor.stop();
                    setPathState(29); // Alternative path
                }
                break;

            case 6:  // Move to transfer position
                intakeServos.transferPosition();
                setPathState(7);
                break;

            case 7:  // Retract linkage
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                setPathState(8);
                break;

            case 8:  // Intake block into bucket
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeMotor.intake();
                    setPathState(9);
                }
                break;

            case 9:  // Stop intake after block is secured
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intakeMotor.stop();
                    setPathState(10);
                }
                break;

            case 10:  // Move to bucket (slides stay up)
                follower.followPath(bucketPath1, 0.6, true);
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                setPathState(11);
                break;

            case 11:  // Deposit block
                if (!follower.isBusy()) {
                    bucketServos.depositPosition();
                    setPathState(12);
                }
                break;

            case 12:  // Prepare for next block
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    bucketServos.transferPosition();
                    follower.followPath(blockPath2);
                    linkageController.setPosition(LinkageController.Position.EXTENDED);
                    setPathState(13);
                }
                break;

            case 13:  // Wait 2s before lowering slides
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    intakeMotor.intake();
                    setPathState(14);
                }
                break;

            case 14:  // Start intake
                if (!follower.isBusy()) {
                    intakeServos.intakePosition();
                    setPathState(15);
                }
                break;

            case 15:  // Wait for block detection
                colorAndDistance.update();
                if (pathTimer.getElapsedTimeSeconds() > 2 || colorAndDistance.getDetectedColor().equals("Yellow")) {
                    intakeMotor.stop();
                    setPathState(16);
                } else if (pathTimer.getElapsedTimeSeconds() > 2 && colorAndDistance.getDetectedColor().equals("None")) {
                    intakeMotor.stop();
                    setPathState(30); // Alternative path
                }
                break;

            case 16:  // Move to transfer position
                intakeServos.transferPosition();
                setPathState(17);
                break;

            case 17:  // Retract linkage after block pickup
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                setPathState(18);
                break;

            case 18:  // Intake block into bucket
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeMotor.intake();
                    setPathState(19);
                }
                break;

            case 19:  // Stop intake after block is secured
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intakeMotor.stop();
                    setPathState(20);
                }
                break;

            case 20:  // Move to bucket with second block (slides stay up)
                follower.followPath(bucketPath2, 0.6, true);
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                setPathState(21);
                break;

            case 21:  // Deposit second block
                if (!follower.isBusy()) {
                    bucketServos.depositPosition();
                    setPathState(22);
                }
                break;

            case 22:  // Move to final block
                follower.followPath(blockPath3, 0.6, true);
                setPathState(23);
                break;

            case 23:  // Wait 2s, then lower slides
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    viperSlides.setTarget(ViperSlides.Target.GROUND);
                    setPathState(24);
                }
                break;

            case 24:  // Start final intake
                intakeMotor.intake();
                setPathState(25);
                break;

            case 25:  // Wait for final block detection
                colorAndDistance.update();
                if (pathTimer.getElapsedTimeSeconds() > 2 || colorAndDistance.getDetectedColor().equals("Yellow")) {
                    intakeMotor.stop();
                    setPathState(26);
                } else if (pathTimer.getElapsedTimeSeconds() > 2 && colorAndDistance.getDetectedColor().equals("None")) {
                    intakeMotor.stop();
                    setPathState(32); // Alternative path for missing 3rd block
                }
                break;

            case 26:  // Move to transfer position
                intakeServos.transferPosition();
                setPathState(27);
                break;

            case 27:  // Retract linkage after block pickup
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                setPathState(28);
                break;

            case 28:  // Intake block into bucket
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    intakeMotor.intake();
                    setPathState(29);
                }
                break;

            case 29:  // Stop intake after block is secured
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intakeMotor.stop();
                    setPathState(30);
                }
                break;

            case 30:  // Move to final bucket (slides stay up)
                follower.followPath(bucketPath3, 0.6, true);
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                setPathState(31);
                break;

            case 31:  // Deposit final block
                if (!follower.isBusy()) {
                    bucketServos.depositPosition();
                    setPathState(32);
                }
                break;

            case 32:  // Wait 2 seconds before lowering slides
                follower.followPath(endPath, 0.8, true);
                setPathState(33);
                break;

            case 33:  // Move to end
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    viperSlides.setTarget(ViperSlides.Target.LOW);
                    setPathState(34);
                }
                break;

            case 34:
                setPathState(-1); // End of autonomous
                break;

            /** ---------------- ALTERNATIVE PATHS ---------------- **/

            case 35:  // Alternative path after missing first block
                follower.followPath(blockPath2Alternative);
                intakeMotor.intake();
                setPathState(17);
                break;

            case 36:  // Alternative path after missing second block
                follower.followPath(blockPath3Alternative);
                setPathState(23);
                break;

            case 37:  // Alternative path after missing third block
                follower.followPath(endPathAlternative, 0.8, true);
                viperSlides.setTarget(ViperSlides.Target.LOW);
                setPathState(34);
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
        clawServo.openPosition();

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
    }
}