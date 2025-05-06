package OpMode.TeleOp;

import static Unused.AutonomousFSM.finalPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit; // ✅ Import for current
import com.qualcomm.robotcore.hardware.DcMotorEx;   // ✅ Required for getCurrent()
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColorAndDistance;
import OpMode.Subsystems.FlapServo;
import OpMode.Subsystems.HandServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServosNEW;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;

@Config
@TeleOp(name = "BlueTeleopNEWERAMP", group = "0")
public class BlueTeleopNEWERAMP extends OpMode {

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_RETRACT,
        OUTAKE_HUMAIN,
        OUTAKE_BLOCK,
        INTAKE_WAITFORBLOCK
    }

    IntakeState intakeState = IntakeState.INTAKE_START;
    ElapsedTime intakeTimer = new ElapsedTime();

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;

    private Follower follower;
    private final Pose startPose = new Pose(finalPose.getX(), finalPose.getY(), Math.toRadians(-90));
    private FtcDashboard dashboard;

    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServosNEW intakeServos;
    private AprilTagProcessor aprilTagProcessor;

    private FlapServo flapServo;
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // ✅ Changed this from DcMotor to DcMotorEx
    private DcMotorEx intakemotor;
    private IntakeMotor intakeMotor;

    private DcMotor extendoMotor;
    private ColorSensor colorSensor;
    private ColorAndDistance colorAndDistance;

    private HandServo handServo;

    private boolean hasRumbled = false;
    private ElapsedTime loopTimer;
    private LinkageController linkageController;
    boolean waitingForExtension = false;

    private boolean previousLeftTriggerState = false;
    private boolean currentLeftTriggerState = false;

    private boolean previousHandState = false;

    private boolean currentHandState = false;
    private boolean isClawOpen = true;

    private boolean isHandOpen = true;

    private boolean previousFlapState = false;

    private boolean currentFlapState = false;


    @Override
    public void init() {
        loopTimer = new ElapsedTime();
        intakeTimer.reset();

        telemetry.addData("startPoseX", startPose.getX());
        telemetry.addData("startPoseY", startPose.getY());
        telemetry.addData("startPoseHeading", startPose.getHeading());

        colorAndDistance = new ColorAndDistance(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        viperSlides = new ViperSlides(
                hardwareMap.get(DcMotorEx.class, "slidemotorleft"),
                hardwareMap.get(DcMotorEx.class, "slidemotorright"),
                hardwareMap.get(TouchSensor.class, "limitSwitch"),
                p, i, d
        );

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        dashboard = FtcDashboard.getInstance();
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // ✅ Initialize with DcMotorEx so we can get current draw
        intakemotor = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intakeMotor = new IntakeMotor(intakemotor);

        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServosNEW(intakeServoRight, intakeServoLeft);
        intakeServos.transferPosition();

        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        linkageController.zeroMotor();

        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);
        bucketServos.transferPosition();

        handServo = new HandServo(hardwareMap.get(Servo.class, "HandServo"));
        handServo.closedPosition();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        double currentAmperage = intakemotor.getCurrent(CurrentUnit.AMPS);

        if (currentAmperage > 5.5) {
            intakeState = IntakeState.OUTAKE_HUMAIN;
        }


        switch (intakeState) {
            case INTAKE_START:
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                intakeMotor.stop();
                intakeServos.transferPosition();
                intakeTimer.reset();
                if (gamepad1.dpad_up) intakeState = IntakeState.INTAKE_EXTEND;
                if (gamepad1.right_bumper) intakeMotor.outtake();
                if (gamepad1.left_bumper) intakeMotor.intake();
                break;

            case INTAKE_EXTEND:
                linkageController.setPosition(LinkageController.Position.EXTENDED);
                if (intakeTimer.seconds() > 1) {
                    intakeMotor.intake();
                    intakeServos.intakePosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_WAITFORBLOCK;
                }
                if (gamepad1.right_bumper) intakeMotor.outtake();
                if (gamepad1.left_bumper) intakeMotor.intake();
                break;

            case INTAKE_WAITFORBLOCK:
                colorAndDistance.update();
                String detectedColor = colorAndDistance.getDetectedColor();
                if (detectedColor.equals("Blue") || detectedColor.equals("Yellow")) {
                    intakeMotor.stop();
                    intakeServos.transferPosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                if (gamepad1.right_bumper) intakeMotor.outtake();
                if (gamepad1.left_bumper) intakeMotor.intake();
                break;

            case INTAKE_RETRACT:
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                if (gamepad1.right_bumper) {
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_HUMAIN;
                } else if (gamepad1.left_bumper) {
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_BLOCK;
                }
                if (gamepad1.dpad_up) intakeState = IntakeState.INTAKE_EXTEND;
                break;

            case OUTAKE_HUMAIN:
                colorAndDistance.update();
                if (intakeTimer.seconds() < 0.75) {
                    intakeMotor.outtake();
                }
                if (intakeTimer.seconds() > 0.75) {
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            case OUTAKE_BLOCK:
                if (intakeTimer.seconds() < 1.5) {
                    intakeMotor.intake();
                }
                if (intakeTimer.seconds() > 1.5) {
                    intakeState = IntakeState.INTAKE_START;
                }
                break;

            default:
                intakeState = IntakeState.INTAKE_START;
        }

        if (gamepad1.dpad_down && intakeState != IntakeState.INTAKE_START) {
            intakeState = IntakeState.INTAKE_START;
        }

        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        colorAndDistance.update();
        String detectedColor = colorAndDistance.getDetectedColor();

        if ((detectedColor.equals("Blue") && !hasRumbled)) {
            gamepad1.rumble(1000);
            hasRumbled = true;
        }
        if (!detectedColor.equals("Blue") && !detectedColor.equals("Yellow")) {
            hasRumbled = false;
        }

        if (viperSlides.getSlidePositionRight() > 1300) {
            if (gamepad1.right_trigger > 0.1) {
                bucketServos.depositPosition();
            } else {
                bucketServos.transferPosition();
            }
        } else {
            bucketServos.transferPosition();
        }

        currentLeftTriggerState = gamepad1.left_trigger > 0.5;
        if (currentLeftTriggerState && !previousLeftTriggerState) {
            if (isClawOpen) {
                clawServo.closedPosition();
            } else {
                clawServo.openPosition();
            }
            isClawOpen = !isClawOpen;
        }
        previousLeftTriggerState = currentLeftTriggerState;

        linkageController.checkForAmperageSpike();
        linkageController.update();
        viperSlides.update();

        if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            viperSlides.resetPosition();
        }
        wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();

        if (gamepad1.y) viperSlides.setTarget(ViperSlides.Target.HIGH);
        if (gamepad1.a && bucketServos.isTransferPosition() && isClawOpen) viperSlides.setTarget(ViperSlides.Target.GROUND);
        if (gamepad1.b && bucketServos.isTransferPosition() ) viperSlides.setTarget(ViperSlides.Target.LOW);
        if (gamepad1.x && bucketServos.isTransferPosition()) viperSlides.setTarget(ViperSlides.Target.MEDIUM);

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

        currentHandState = gamepad1.touchpad;
        if (currentHandState && !previousHandState) {
            if (isHandOpen) {
                handServo.openPosition();
            } else {
                handServo.closedPosition();
            }
            isHandOpen = !isHandOpen;
        }
        previousHandState = currentHandState;


        // ✅ Telemetry showing intake motor current
        telemetry.addData("Intake Motor Amperage (A)", intakemotor.getCurrent(CurrentUnit.AMPS));

        telemetry.addData("Loop Time (ms)", loopTime);
        telemetry.addData("Slide Position Left", viperSlides.getSlidePositionLeft());
        telemetry.addData("Slide Position Right", viperSlides.getSlidePositionRight());
        telemetry.addData("Slide Target", viperSlides.getTarget());
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("Current Position", linkageController.getCurrentPosition());
        telemetry.addData("Target Position", linkageController.getTargetPosition());
        telemetry.addData("At Target", linkageController.isAtTarget());
        telemetry.addData("Is Extended", linkageController.isExtended());
        telemetry.addData("Is Retracted", linkageController.isRetracted());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("intake State", intakeState);
        telemetry.update();
    }
}
