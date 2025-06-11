package OpMode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import static OpMode.Subsystems.Sequence.IntakeState.INTAKE_START;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sequence {

    private DcMotorEx intakeMotor;
    private Servo intakeServoRight;


    private final IntakeTest intakeTest = new IntakeTest(telemetry);

    IntakeState intakeState = IntakeState.INTAKE_START;

    public enum IntakeState {
        INTAKE_START,
    }


    ElapsedTime intakeTimer = new ElapsedTime();


    public void Loop() {
        intakeTest.intake();
        if (intakeTimer.seconds()>2){
            intakeTest.stop();
            if (intakeTimer.seconds()>3)
            intakeTimer.reset();
        }
        }


    }