package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class SwerveDriveOpMode extends LinearOpMode {
    private DcMotor FLdrive, FRdrive, BLdrive, BRdrive;
    private CRServoImplEx FLsteer, FRsteer, BLsteer, BRsteer;
    private AnalogInput FLencoder, FRencoder, BLencoder, BRencoder;

    public static boolean MOTOR_FLIPPING = true;

    public boolean FLflipped = false;
    public boolean FRflipped = false;
    public boolean BLflipped = false;
    public boolean BRflipped = false;


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

        FLsteer = hardwareMap.get(CRServoImplEx.class, "FLsteer");
        FRsteer = hardwareMap.get(CRServoImplEx.class, "FRsteer");
        BLsteer = hardwareMap.get(CRServoImplEx.class, "BLsteer");
        BRsteer = hardwareMap.get(CRServoImplEx.class, "BRsteer");

        // axon servos have a PWM range of 500us-2500us
        FLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
            double bra = Math.atan2(a,d) * 180/Math.PI;
            double bla = Math.atan2(a,c) * 180/Math.PI;

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

            // Wheel positions
            double FLpos = FLencoder.getVoltage() / 3.3 * 360 - FLoffset;
            double FRpos = FRencoder.getVoltage() / 3.3 * 360 - FRoffset;
            double BLpos = BLencoder.getVoltage() / 3.3 * 360 - BLoffset;
            double BRpos = BRencoder.getVoltage() / 3.3 * 360 - BRoffset;

            // Errors
            double FLerror = normalizeDegrees(fla - FLpos);
            double FRerror = normalizeDegrees(fra - FRpos);
            double BLerror = normalizeDegrees(bla - BLpos);
            double BRerror = normalizeDegrees(bra - BRpos);

            // flip shortcut
            if (MOTOR_FLIPPING && Math.abs(FLerror) > Math.PI / 2) {
                fla = normalizeRadians(fla - Math.PI);
                FLflipped = true;
            } else {
                FLflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(FRerror) > Math.PI / 2) {
                fra = normalizeRadians(fra - Math.PI);
                FRflipped = true;
            } else {
                FRflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(BLerror) > Math.PI / 2) {
                bla = normalizeRadians(bla - Math.PI);
                BLflipped = true;
            } else {
                BLflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(BRerror) > Math.PI / 2) {
                bra = normalizeRadians(bra - Math.PI);
                BRflipped = true;
            } else {
                BRflipped = false;
            }

            FLerror = normalizeDegrees(fla - FLpos);
            FRerror = normalizeDegrees(fra - FRpos);
            BLerror = normalizeDegrees(bla - BLpos);
            BRerror = normalizeDegrees(bra - BRpos);


            // Wheel PID angles
            double FLoutput = FLpid.calculate(0, FLerror);
            double FRoutput = FRpid.calculate(0, FRerror);
            double BLoutput = BLpid.calculate(0, BLerror);
            double BRoutput = BRpid.calculate(0, BRerror);

            FLsteer.setPower(FLoutput);
            FRsteer.setPower(FRoutput);
            BLsteer.setPower(BLoutput);
            BRsteer.setPower(BRoutput);





            // Set drive powers
            if (FLflipped)
                FLdrive.setPower(-fls);
            else
                FLdrive.setPower(fls);

            if (FRflipped)
                FRdrive.setPower(-frs);
            else
                FRdrive.setPower(frs);

            if (BLflipped)
                BLdrive.setPower(-rls);
            else
                BLdrive.setPower(rls);

            if (BRflipped)
                BRdrive.setPower(-rrs);
            else
                BRdrive.setPower(rrs);




            // telemetry wheel speeds
            telemetry.addData("FLspeed", fls);
            telemetry.addData("FRspeed", frs);
            telemetry.addData("BLspeed", rls);
            telemetry.addData("BRspeed", rrs);

            // telemetry wheel angles
            telemetry.addData("FLangle", fla);
            telemetry.addData("FRangle", fra);
            telemetry.addData("BLangle", bla);
            telemetry.addData("BRangle", bra);

            // telemetry flipped values
            telemetry.addData("FLflipped", FLflipped);
            telemetry.addData("FRflipped", FRflipped);
            telemetry.addData("BLflipped", BLflipped);
            telemetry.addData("BRflipped", BRflipped);


            telemetry.update();
        }
    }
}
