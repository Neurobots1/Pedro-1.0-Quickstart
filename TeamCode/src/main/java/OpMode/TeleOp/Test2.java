package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import OpMode.Subsystems.IntakeTest;


@TeleOp(name = "Test2", group = "Test")
public class Test2 extends OpMode {
    private IntakeTest intakeTest;

    @Override
    public void init(){
        intakeTest.setup(hardwareMap);
    }

    @Override
    public void loop(){
        if (gamepad1.a){
            intakeTest.intake();
        }

    }
}


