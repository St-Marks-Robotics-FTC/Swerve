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
public class SlewTest extends LinearOpMode {




    public static double fwd_rate_limit = 1;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        SlewRateLimiter fwdSlew = new SlewRateLimiter(fwd_rate_limit);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double fwd = -gamepad1.left_stick_y;


            telemetry.addData("Joystick Y: ", fwd);
            telemetry.addData("Slew Y: ", fwdSlew.calculate(fwd));

            telemetry.update();
        }
    }
}
