package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;


//@Disabled
@Config
@TeleOp
public class CRservoTest extends LinearOpMode {

    public static double servoPos = 0;

    public static String servoName = "FRsteer";
    public static String encoderName = "FRencoder";

    private CRServoImplEx servo;
    private AnalogInput servoEncoder;

    public static double kP = .04; //1/25
    public static double kI = 0.0;
    public static double kD = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(CRServoImplEx.class, servoName);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servoEncoder = hardwareMap.get(AnalogInput.class, encoderName);


        PIDController servoPID = new PIDController(kP, kI, kD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            servoPID.setPID(kP, kI, kD);

            double output = servoPID.calculate(servoEncoder.getVoltage() / 3.3 * 360, servoPos);

            if (gamepad1.right_trigger > 0.1) {
                servo.setPower(gamepad1.right_trigger);
            } else {
                servo.setPower(output);
            }



            telemetry.addData("Servo Set Position: ", servoPos);
            telemetry.addData("Servo Current Position: ", servoEncoder.getVoltage() / 3.3 * 360);

            telemetry.addData("Servo Power: ", output);

            telemetry.update();
        }
    }
}
