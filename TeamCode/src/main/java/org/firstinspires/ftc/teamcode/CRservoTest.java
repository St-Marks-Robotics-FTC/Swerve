package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
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

    private CRServoImplEx servo;
    private AnalogInput servoEncoder;

    public static double kP = 0.5;
    public static double kI = 0.0;
    public static double kD = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(CRServoImplEx.class, servoName);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        servoEncoder = hardwareMap.get(AnalogInput.class, "FRencoder");


        PIDController servoPID = new PIDController(kP, kI, kD);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double output = servoPID.calculate(servoEncoder.getVoltage() / 3.3 * 360, servoPos);

            servo.setPower(output);



            telemetry.addData("Servo Set Position: ", servoPos);
            telemetry.addData("Servo Current Position: ", servoEncoder.getVoltage() / 3.3 * 360);

            telemetry.update();
        }
    }
}
