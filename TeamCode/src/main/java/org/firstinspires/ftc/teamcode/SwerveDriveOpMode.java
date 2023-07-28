package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class SwerveDriveOpMode extends LinearOpMode {
    private DcMotor FLdrive, FRdrive, BLdrive, BRdrive;
    private CRServo FLsteer, FRsteer, BLsteer, BRsteer;
    private AnalogInput FLencoder, FRencoder, BLencoder, BRencoder;

    public static double FLoffset = 0.0;
    public static double FRoffset = 0.0;
    public static double BLoffset = 0.0;
    public static double BRoffset = 0.0;


    // The IMU sensor object
    IMU imu;


    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;


    @Override
    public void runOpMode() {

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Initialize hardware components
        FLdrive = hardwareMap.get(DcMotor.class, "FLdrive");
        FRdrive = hardwareMap.get(DcMotor.class, "FRdrive");
        BLdrive = hardwareMap.get(DcMotor.class, "BLdrive");
        BRdrive = hardwareMap.get(DcMotor.class, "BRdrive");
        
        // velocity pid
        FLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLsteer = hardwareMap.get(CRServo.class, "FLsteer");
        FRsteer = hardwareMap.get(CRServo.class, "FRsteer");
        BLsteer = hardwareMap.get(CRServo.class, "BLsteer");
        BRsteer = hardwareMap.get(CRServo.class, "BRsteer");

        FLencoder = hardwareMap.get(AnalogInput.class, "FLencoder");
        FRencoder = hardwareMap.get(AnalogInput.class, "FRencoder");
        BLencoder = hardwareMap.get(AnalogInput.class, "BLencoder");
        BRencoder = hardwareMap.get(AnalogInput.class, "BRencoder");


        PIDController FLpid = new PIDController(kP, kI, kD);
        PIDController FRpid = new PIDController(kP, kI, kD);
        PIDController BLpid = new PIDController(kP, kI, kD);
        PIDController BRpid = new PIDController(kP, kI, kD);

        waitForStart();

        while (opModeIsActive()) {

            double fwd = -gamepad1.left_stick_y; // Pushing joystick up is negative
            double str = gamepad1.left_stick_x; // Pushing joystick to the right is positive
            double rcw = gamepad1.right_stick_x; // Clockwise rotation is positive

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            // field centric
            double temp = (fwd*Math.cos(orientation.getYaw(AngleUnit.RADIANS))) + str*Math.sin(orientation.getYaw(AngleUnit.RADIANS));
            double str2 = (-fwd*Math.sin(orientation.getYaw(AngleUnit.RADIANS))) + str*Math.cos(orientation.getYaw(AngleUnit.RADIANS));
            double fwd2 = temp;

            // units doesn't matter as long as they are the same
            double wheelbase = 30.0;
            double trackwidth = 24.0;
            double r = Math.sqrt((wheelbase*wheelbase) + (trackwidth*trackwidth));

            double a = str2 - rcw * (wheelbase/r);
            double b = str2 + rcw * (wheelbase/r);
            double c = fwd2 - rcw * (trackwidth/r);
            double d = fwd2 + rcw * (trackwidth/r);

            // wheel speeds
            double frs = Math.sqrt(b*b + c*c);
            double fls = Math.sqrt(b*b + d*d);
            double rls = Math.sqrt(a*a + d*d);
            double rrs = Math.sqrt(a*a + c*c);

            // wheel angles in degrees +- 180
            double fra = Math.atan2(b,c) * 180/Math.PI;
            double fla = Math.atan2(b,d) * 180/Math.PI;
            double rra = Math.atan2(a,d) * 180/Math.PI;
            double rla = Math.atan2(a,c) * 180/Math.PI;

            // Normalize wheel speeds
            double max = frs;
            if(fls>max){
                max = fls;
            }
            if(rls>max){
                max = rls;
            }
            if(rrs>max){
                max = rrs;
            }
            if(max>1){
                frs/=max;
                fls/=max;
                rrs/=max;
                rls/=max;
            }


            // Set wheel angles
            FLpid.setPID(kP, kI, kD);
            FRpid.setPID(kP, kI, kD);
            BLpid.setPID(kP, kI, kD);
            BRpid.setPID(kP, kI, kD);

            double FLoutput = FLpid.calculate(FLencoder.getVoltage() / 3.3 * 360 - FLoffset, fla);
            double FRoutput = FRpid.calculate(FRencoder.getVoltage() / 3.3 * 360 - FRoffset, fra);
            double BLoutput = BLpid.calculate(BLencoder.getVoltage() / 3.3 * 360 - BLoffset, rla);
            double BRoutput = BRpid.calculate(BRencoder.getVoltage() / 3.3 * 360 - BRoffset, rra);

            FLsteer.setPower(FLoutput);
            FRsteer.setPower(FRoutput);
            BLsteer.setPower(BLoutput);
            BRsteer.setPower(BRoutput);





            // Set drive powers
            FLdrive.setPower(fls);
            FRdrive.setPower(frs);
            BLdrive.setPower(rls);
            BRdrive.setPower(rrs);




            // telemetry wheel speeds
            telemetry.addData("FLspeed", fls);
            telemetry.addData("FRspeed", frs);
            telemetry.addData("RLspeed", rls);
            telemetry.addData("RRspeed", rrs);

            // telemetry wheel angles
            telemetry.addData("FLangle", fla);
            telemetry.addData("FRangle", fra);
            telemetry.addData("RLangle", rla);
            telemetry.addData("RRangle", rra);


            telemetry.update();
        }
    }
}
