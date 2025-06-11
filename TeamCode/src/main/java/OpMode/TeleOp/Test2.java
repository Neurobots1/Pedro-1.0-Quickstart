package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import OpMode.Subsystems.IntakeTest;
import OpMode.Subsystems.Sequence;


@TeleOp(name = "Test2", group = "Test")
public class Test2 extends OpMode {

    private DcMotorEx intakeMotor;
    private Servo intakeServoRight;

    public enum IntakeState{
        INTAKE_START
    }


    private final Sequence sequence= new Sequence();
    private final IntakeTest intakeTest= new IntakeTest(telemetry);

    @Override
    public void init(){
        intakeTest.setup(hardwareMap);
    }

    @Override
    public void loop() {
        sequence.Loop();

    }
}


