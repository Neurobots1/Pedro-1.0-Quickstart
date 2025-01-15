package OpMode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Linkage Test TeleOp", group = "Test")
public class LinkageTestTeleOp extends OpMode {

    private LinkageController linkageController;

    @Override
    public void init() {
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        // Start the zeroing process
        linkageController.zeroMotor();
    }

    @Override
    public void loop() {
        // Check for amperage spike and handle zeroing
        linkageController.checkForAmperageSpike();

        if (gamepad1.dpad_up) {
            linkageController.setPosition(LinkageController.Position.EXTENDED);
        } else if (gamepad1.dpad_down) {
            linkageController.setPosition(LinkageController.Position.RETRACTED);
        }

        linkageController.update();

        telemetry.addData("Current Position", linkageController.getCurrentPosition());
        telemetry.addData("Target Position", linkageController.getTargetPosition());
        telemetry.addData("At Target", linkageController.isAtTarget());
        telemetry.update();
    }
}
