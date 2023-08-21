package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp()
public class LoopTimes extends LinearOpMode {


    final ElapsedTime loopTimer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize hardware components
        DcMotor FRdrive = hardwareMap.get(DcMotor.class, "FRdrive");
        DcMotor FLdrive = hardwareMap.get(DcMotor.class, "FLdrive");
        DcMotor BLdrive = hardwareMap.get(DcMotor.class, "BLdrive");
        DcMotor BRdrive = hardwareMap.get(DcMotor.class, "BRdrive");

        // velocity pid
        FRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CRServoImplEx FRsteer = hardwareMap.get(CRServoImplEx.class, "FRsteer");
        CRServoImplEx FLsteer = hardwareMap.get(CRServoImplEx.class, "FLsteer");
        CRServoImplEx BLsteer = hardwareMap.get(CRServoImplEx.class, "BLsteer");
        CRServoImplEx BRsteer = hardwareMap.get(CRServoImplEx.class, "BRsteer");

        // axon servos have a PWM range of 500us-2500us
        FRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));

        AnalogInput FRencoder = hardwareMap.get(AnalogInput.class, "FRencoder");
        AnalogInput FLencoder = hardwareMap.get(AnalogInput.class, "FLencoder");
        AnalogInput BLencoder = hardwareMap.get(AnalogInput.class, "BLencoder");
        AnalogInput BRencoder = hardwareMap.get(AnalogInput.class, "BRencoder");



        waitForStart();
        loopTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {







            telemetry.addData("Loop Time in Ms: ", loopTimer.milliseconds());
            telemetry.addData("Loop Frequency: ", 1000/loopTimer.milliseconds());
            telemetry.update();
            loopTimer.reset();
        }
    }
}
