package OpMode.TeleOp.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import OpMode.Subsystems.FlapServo;


@TeleOp(name = "FlapServoTest", group = "Test")
public class FlapServoTest extends OpMode {

    private FlapServo flapServo;
    private Servo FlapServo;

    @Override
    public void init() {
        FlapServo = hardwareMap.get(Servo.class, "FlapServo");
        flapServo = new FlapServo(FlapServo);

    }

    @Override
    public void loop(){

        if (gamepad1.a){
            flapServo.FlapClose();
        }

        if (gamepad1.b){
            flapServo.FlapOpen();
        }

    }
}
