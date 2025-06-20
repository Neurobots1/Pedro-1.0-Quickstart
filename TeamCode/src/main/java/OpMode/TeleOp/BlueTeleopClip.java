package OpMode.TeleOp;

import static OpMode.Autonomous.AutonomousFSM.finalPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColorAndDistance;
import OpMode.Subsystems.FlapServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServosNEW;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "BlueTeleopClip", group = "0")
public class BlueTeleopClip extends OpMode {

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_RETRACT,
        OUTAKE_HUMAIN,
        OUTAKE_BLOCK,

        INTAKE_WAITFORBLOCK

    }

    public enum SlideState {
        IDLE,
        Drop,
        BlockInBucket,
        BucketScoring,
        Descending,
        FlapClosed,
        SlideMID,
    }

    IntakeState intakeState = IntakeState.INTAKE_START;

    SlideState slideState = SlideState.IDLE;
    ElapsedTime intakeTimer = new ElapsedTime();

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;
    // PedroPathing Teleop
    private Follower follower;
    private final Pose startPose = new Pose(finalPose.getX(), finalPose.getY(),finalPose.getHeading());
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private Timer SlideTimer;
    private IntakeServosNEW intakeServos;
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // Intake Motor and Color Sensor
    private DcMotor intakemotor;
    private IntakeMotor intakeMotor;
    private DcMotor extendoMotor;
    private ColorSensor colorSensor;
    private ColorAndDistance colorAndDistance;;

    private FlapServo flapServo;

    private Servo FlapServo;

    private boolean hasRumbled = false;
    private ElapsedTime loopTimer;

    private LinkageController linkageController;
    boolean waitingForExtension = false;

    // Left Trigger Rising Edge Detection
    private boolean previousRightTriggerState = false;
    private boolean currentRightTriggerState = false;
    private boolean isClawOpen = true;

    private boolean isFlapOpen = false;

    private boolean previousFlapState = false;

    private boolean currentFlapState = false;

    @Override
    public void init() {

        // Initialize the loop timer
        loopTimer = new ElapsedTime();
        SlideTimer = new Timer();
        intakeTimer.reset();

        //startPose
        telemetry.addData("startPoseX", startPose.getX());
        telemetry.addData("startPoseY", startPose.getY());
        telemetry.addData("startPoseHeading", startPose.getHeading());

        // Corrected Color Sensor Initialization
        colorAndDistance = new ColorAndDistance(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        // Initialize Viper Slide
        viperSlides = new ViperSlides(
                hardwareMap.get(DcMotorEx.class, "slidemotorleft"),
                hardwareMap.get(DcMotorEx.class, "slidemotorright"),
                hardwareMap.get(TouchSensor.class, "limitSwitch"),
                p, i, d
        );

        // Initialize Pedro follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize REV Touch Sensor
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // Initialize motors and sensors
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));

        FlapServo = hardwareMap.get(Servo.class, "FlapServo");
        flapServo = new FlapServo(FlapServo);


        // Initialize intake servos
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServosNEW(intakeServoRight, intakeServoLeft);
        intakeServos.transferPosition();

        // Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");
        linkageController.zeroMotor();

        // Initialize claw servo
        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        // Initialize Bucket Servo
        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);
        bucketServos.transferPosition();
    }

    @Override
    public void loop() {

        currentRightTriggerState = gamepad1.right_trigger > 0.5;  // Detect if the left trigger is pressed
        if (currentRightTriggerState && !previousRightTriggerState) {  // Rising edge
            // Toggle claw position on rising edge
            if (isFlapOpen) {
                flapServo.FlapClose();  // Close the claw
            } else {
                flapServo.FlapOpen();  // Open the claw
            }
            // Flip the claw state
            isFlapOpen = !isFlapOpen;
        }
        previousRightTriggerState = currentRightTriggerState;

        if (gamepad1.y){
            slideState = SlideState.BlockInBucket;
        }

        if (gamepad1.a && bucketServos.isTransferPosition()) {
            slideState = SlideState.SlideMID;
        }

        switch (intakeState) {


            case INTAKE_START:
               linkageController.setPosition(LinkageController.Position.RETRACTED);
               intakeMotor.stop();
               intakeServos.transferPosition();
               intakeTimer.reset();
               if (gamepad1.dpad_up) {
                   intakeState = IntakeState.INTAKE_EXTEND;
               }

               if (gamepad1.left_bumper){
                   intakeMotor.outtake();
               }

                break;

            case INTAKE_EXTEND:
                linkageController.setPosition(LinkageController.Position.EXTENDED);
                if (intakeTimer.seconds()>1){
                    intakeMotor.intake();
                    intakeServos.intakePosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_WAITFORBLOCK;
                }
                if (gamepad1.left_bumper){
                    intakeMotor.outtake();
                }

                break;

            case INTAKE_WAITFORBLOCK:
                colorAndDistance.update();

                String detectedColor = colorAndDistance.getDetectedColor();

                if (detectedColor.equals("Blue" ) || detectedColor.equals("Yellow") ){
                    intakeMotor.stop();
                    intakeServos.transferPosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                if (gamepad1.right_bumper) {
                    intakeMotor.outtake();
                }

                if (gamepad1.left_bumper){
                    intakeMotor.intake();
                }



                break;

            case INTAKE_RETRACT:
                linkageController.setPosition(LinkageController.Position.RETRACTED);

                if (gamepad1.right_bumper){
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_HUMAIN;
                } else if (gamepad1.left_bumper) {
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_BLOCK;

                }

                if (gamepad1.dpad_up) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                break;

            case OUTAKE_HUMAIN:
                colorAndDistance.update();
                if (intakeTimer.seconds()<1.5 ){
                   intakeMotor.outtake();
                }

                if (intakeTimer.seconds()>1.5){
                    intakeState = IntakeState.INTAKE_START;
                }




                break;

            case OUTAKE_BLOCK:
                if (intakeTimer.seconds()<1.5){
                    intakeMotor.intake();

                }

                if (intakeTimer.seconds()>1.5){
                    intakeState = IntakeState.INTAKE_START;

                }


                break;

            default:
                // should never be reached, as intakeState should never be null
                intakeState = IntakeState.INTAKE_START;

        }

        switch (slideState) {

            case IDLE :


                break;


            case BlockInBucket:
                viperSlides.setTarget(ViperSlides.Target.HIGH);
                if (viperSlides.getSlidePositionRight() > 1000){
                    slideState = SlideState.BucketScoring;
                    bucketServos.depositPosition();
                }
                break;

            case BucketScoring:
                if (gamepad1.right_trigger > 0.5){
                    isFlapOpen = true;
                    SlideTimer.resetTimer();
                    slideState = SlideState.Descending;
                    // flapServo.FlapOpen();
                }
                break;

            case Descending:

            if (SlideTimer.getElapsedTimeSeconds()>1){
                slideState = SlideState.FlapClosed;
            }
                break;
            case FlapClosed:
            isFlapOpen = false;
            bucketServos.transferPosition();

                break;
            case SlideMID:
            viperSlides.setTarget(ViperSlides.Target.GROUND);
            slideState = SlideState.IDLE;
                break;

            default:

                slideState = SlideState.IDLE;



        }

        if (gamepad1.dpad_down && intakeState != IntakeState.INTAKE_START) {
            intakeState = IntakeState.INTAKE_START;
        }
        // Measure loop time
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        // TeleOp movement
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        // Update Color Sensor
        colorAndDistance.update();

        String detectedColor = colorAndDistance.getDetectedColor();

        // Rumble for Blue Detection





        linkageController.checkForAmperageSpike();
        linkageController.update();

        // Viper Slide Control (Predefined Targets)
        viperSlides.update();

        if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            viperSlides.resetPosition();
        }

        wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();

        if (gamepad1.y) {
            viperSlides.setTarget(ViperSlides.Target.MEDIUM);
        }
        if (gamepad1.a && bucketServos.isTransferPosition() && isClawOpen) {
            viperSlides.setTarget(ViperSlides.Target.GROUND);
        }
        if (gamepad1.b && bucketServos.isTransferPosition()) {
            viperSlides.setTarget(ViperSlides.Target.LOW);
        }
        if (gamepad1.x && bucketServos.isTransferPosition()) {
            viperSlides.setTarget(ViperSlides.Target.HIGH);

        }

        if (gamepad1.touchpad){
            viperSlides.setTarget(ViperSlides.Target.LEVEL1);
        }

        // Home Slides
        if (gamepad1.options) {
            viperSlides.setPIDEnabled(false);

            while (!viperSlides.isLimitSwitchPressed()) {
                viperSlides.setSlidePower(-1.0);
            }

            viperSlides.setSlidePower(0);
            viperSlides.resetPosition();
            viperSlides.setPIDEnabled(true);
        }

        if (gamepad1.back) {
            Pose currentPose = follower.getPose();
            Pose newPose = new Pose(currentPose.getX(), currentPose.getY(), 0);
            follower.setPose(newPose);
        }

        currentFlapState = gamepad1.left_trigger > 0.5;
        if (currentFlapState && !previousFlapState) {
            isFlapOpen = !isFlapOpen;
        }
        previousFlapState = currentFlapState;

        if (isFlapOpen){
            flapServo.FlapOpen();
        } else {
            flapServo.FlapClose();
        }


        // Telemetry for debugging and visualization
        telemetry.addData("Loop Time (ms)", loopTime);  // Show the loop time in ms
        telemetry.addData("Slide Position Left", viperSlides.getSlidePositionLeft());
        telemetry.addData("Slide Position Right", viperSlides.getSlidePositionRight());
        telemetry.addData("Slide Target", viperSlides.getTarget());
        telemetry.addData("Detected Color", colorAndDistance.getDetectedColor());
        //
        telemetry.addData("Current Position", linkageController.getCurrentPosition());
        telemetry.addData("Target Position", linkageController.getTargetPosition());
        telemetry.addData("At Target", linkageController.isAtTarget());
        telemetry.addData("Is Extended", linkageController.isExtended());
        telemetry.addData("Is Retracted", linkageController.isRetracted());
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("intake State", intakeState);
        telemetry.addData("slide state", slideState);
        telemetry.update();
    }
    }
