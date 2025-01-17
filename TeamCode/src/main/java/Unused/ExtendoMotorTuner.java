package Unused;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;  // Make sure to import the FtcDashboard class

@TeleOp
@Config // Makes the PID parameters configurable in FTC Dashboard
public class ExtendoMotorTuner extends OpMode {

    // PIDF parameters to tune in FTC Dashboard
    public static double p = 0.005;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0;
    public static int targetPosition = 0;  // Target position to reach

    private DcMotorEx extendoMotor;
    private PIDController controller;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize motor
        extendoMotor = hardwareMap.get(DcMotorEx.class, "extendoMotor");

        // Initialize the PID controller with the configurable PID values
        controller = new PIDController(p, i, d);

        // Configure the motor
        extendoMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update the PID controller with the current configurable values
        controller.setPID(p, i, d);

        // Get the current position of the motor
        int currentPosition = extendoMotor.getCurrentPosition();

        // Compute the PID output
        double pidOutput = controller.calculate(currentPosition, targetPosition);

        // Apply feedforward (f) value
        double power = pidOutput + f;

        // Set the motor power
        extendoMotor.setPower(power);

        // Telemetry for Driver Station
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("Motor Power", power);
        telemetry.update();

        // Telemetry for FTC Dashboard
        dashboard.getTelemetry().addData("Target Position", targetPosition);
        dashboard.getTelemetry().addData("Current Position", currentPosition);
        dashboard.getTelemetry().addData("PID Output", pidOutput);
        dashboard.getTelemetry().addData("Motor Power", power);
        dashboard.getTelemetry().addData("Position Error", targetPosition - currentPosition);
        dashboard.getTelemetry().update();
    }
}
