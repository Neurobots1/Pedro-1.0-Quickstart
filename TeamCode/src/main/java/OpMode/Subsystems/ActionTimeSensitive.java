package OpMode.Subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionTimeSensitive {

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;
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



    private Timer timer;

    public void Bucket(){
        timer.resetTimer();
        viperSlides.setTarget(ViperSlides.Target.HIGH);
        if (viperSlides.isAtTargetPosition(ViperSlides.Target.HIGH)){
            bucketServos.depositPosition();
            if (timer.getElapsedTimeSeconds()>2 && timer.getElapsedTimeSeconds()<2.1){
                bucketServos.transferPosition();
            }
        }
    }

    public void Outake() {
        timer.resetTimer();
        intakeServos.transferPosition();
        linkageController.setPosition(LinkageController.Position.RETRACTED);
        if (timer.getElapsedTimeSeconds() > 0.5 && timer.getElapsedTimeSeconds() < 1.5) {
            intakeMotor.intake();
        }
        if (timer.getElapsedTimeSeconds() > 1.6 && timer.getElapsedTimeSeconds() < 1.7) {
            intakeMotor.stop();
        }

    }

    public void Intake(){
        linkageController.setPosition(LinkageController.Position.EXTENDED);

    }


}
