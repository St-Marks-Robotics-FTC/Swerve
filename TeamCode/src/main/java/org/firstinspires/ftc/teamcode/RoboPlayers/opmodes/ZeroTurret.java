package org.firstinspires.ftc.teamcode.RoboPlayers.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.RoboPlayers.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.RoboPlayers.subsystems.Intake;
import org.firstinspires.ftc.teamcode.RoboPlayers.subsystems.Outtake;

@TeleOp
public class ZeroTurret extends LinearOpMode {

    private DcMotor turret, intakeSlide, outtakeSlide1, outtakeSlide2;

    Drivetrain drive = new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        drive.init(hardwareMap);

        turret = hardwareMap.dcMotor.get("turret");
        intakeSlide = hardwareMap.dcMotor.get("intakeslide");

        outtakeSlide1 = hardwareMap.dcMotor.get("outtake1");
        outtakeSlide2 = hardwareMap.dcMotor.get("outtake2");

        outtakeSlide2.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Drive
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            drive.driveReverse(forward , strafe , turn );


            



            if(gamepad1.left_bumper) {
                turret.setPower(.4);
            } else if (gamepad1.right_bumper) {
                turret.setPower(-.4);
            } else {
                turret.setPower(0);
            }

            if(gamepad1.left_trigger > 0.5) {
                intakeSlide.setPower(.3);
            } else if (gamepad1.right_trigger > 0.5) {
                intakeSlide.setPower(-.3);
            } else {
                intakeSlide.setPower(0);
            }

            if(gamepad1.dpad_up) {
                outtakeSlide1.setPower(.3);
                outtakeSlide2.setPower(.3);

            } else if (gamepad1.dpad_down) {
                outtakeSlide1.setPower(-.3);
                outtakeSlide2.setPower(-.3);
            } else {
                outtakeSlide1.setPower(0);
                outtakeSlide2.setPower(0);
            }


            telemetry.addData("Turret: ", turret.getCurrentPosition());
            telemetry.addData("Intake: ", intakeSlide.getCurrentPosition());
            telemetry.addData("Outtake1: ", outtakeSlide1.getCurrentPosition());
            telemetry.addData("Outtake2: ", outtakeSlide2.getCurrentPosition());

            telemetry.update();
        }
    }
}
