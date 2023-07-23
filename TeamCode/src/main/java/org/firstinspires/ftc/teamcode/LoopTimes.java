package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp()
public class LoopTimes extends LinearOpMode {


    ElapsedTime loopTimer = new ElapsedTime();

    double clawPos = 0;
    int slidePos = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad currentGamepad1 = new Gamepad();

        loopTimer.reset();

        waitForStart();
        loopTimer.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            telemetry.addData("Loop Time: ", loopTimer.milliseconds());
            telemetry.addData("Loop Frequency: ", 1/loopTimer.milliseconds());
            telemetry.update();
            loopTimer.reset();
        }
    }
}
