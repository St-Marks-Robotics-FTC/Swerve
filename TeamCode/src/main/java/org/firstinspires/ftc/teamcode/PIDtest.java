package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

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
public class PIDtest extends LinearOpMode {

    public static double targetPos = 180;

    public static String servoName = "FRsteer";
    public static String encoderName = "FRencoder";

    private CRServoImplEx servo;
    private AnalogInput servoEncoder;

    public static double kP = .02; //1/25
    public static double kI = 0.0;
    public static double kD = 0;

    public static boolean flipEncoder = false;



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

            double curPos = servoEncoder.getVoltage() / 3.3 * 360;
            if (flipEncoder) {
                curPos = 360 - curPos;
            }


            double output = servoPID.calculate(curPos, targetPos);


            // + is clockwise    - is anticlockwise
            if (gamepad1.right_trigger > 0.1) {
                output = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0.1) {
                output = -gamepad1.left_trigger;
            }

            servo.setPower(output);



            telemetry.addData("Servo Target Position: ", targetPos);
            telemetry.addData("Servo Current Position: ", curPos);

            telemetry.addData("Servo Power: ", output);

            telemetry.update();
        }
    }
}
