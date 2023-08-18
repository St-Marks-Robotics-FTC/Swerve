package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Config
@TeleOp
public class SwerveDriveOpMode extends LinearOpMode {
    public static boolean threading = false;
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;

    ElapsedTime loopTimer = new ElapsedTime();

    private DcMotor FLdrive, FRdrive, BLdrive, BRdrive;
    private CRServoImplEx FLsteer, FRsteer, BLsteer, BRsteer;
    private AnalogInput FLencoder, FRencoder, BLencoder, BRencoder;

    public static boolean MOTOR_FLIPPING = false;
    public static double joystickLimit = .001;

    public boolean FLflipped = false;
    public boolean FRflipped = false;
    public boolean BLflipped = false;
    public boolean BRflipped = false;

    public static boolean FLflippedEnc = false;
    public static boolean FRflippedEnc = false;
    public static boolean BLflippedEnc = false;
    public static boolean BRflippedEnc = false;


    public static double FLoffset = 69;
    public static double FRoffset = 300;
    public static double BLoffset = 129;
    public static double BRoffset = 8;


    // The IMU sensor object
    //IMU imu;


    public static double kP = .02; //1/25
    public static double kI = 0.0;
    public static double kD = 0;



    public static boolean slew = false;
    public static double fw_r = 4;
    public static double str_r = 4;
    public static double rot_r = 4;



    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if (threading) {
            synchronized (imuLock) {
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//            imu.initialize(parameters);

                imu = hardwareMap.get(IMU.class, "imu");

                RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
                RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

                // Now initialize the IMU with this mounting orientation
                // Note: if you choose two conflicting directions, this initialization will cause a code exception.
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                // Reset Yaw
                imu.resetYaw();
            }

            imuThread = new Thread(() -> {
                while (!isStopRequested() && opModeIsActive()) {
                    synchronized (imuLock) {
                        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    }
                }
            });
            imuThread.start();

            telemetry.addLine("IMU thread started");
            telemetry.update();
        } else {
            // Retrieve and initialize the IMU.
            // This sample expects the IMU to be in a REV Hub and named "imu".
            imu = hardwareMap.get(IMU.class, "imu");

            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

            // Now initialize the IMU with this mounting orientation
            // Note: if you choose two conflicting directions, this initialization will cause a code exception.
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            // Reset Yaw
            imu.resetYaw();
        }






        // Initialize hardware components
        FRdrive = hardwareMap.get(DcMotor.class, "FRdrive");
        FLdrive = hardwareMap.get(DcMotor.class, "FLdrive");
        BLdrive = hardwareMap.get(DcMotor.class, "BLdrive");
        BRdrive = hardwareMap.get(DcMotor.class, "BRdrive");
        
        // velocity pid
        FRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FRsteer = hardwareMap.get(CRServoImplEx.class, "FRsteer");
        FLsteer = hardwareMap.get(CRServoImplEx.class, "FLsteer");
        BLsteer = hardwareMap.get(CRServoImplEx.class, "BLsteer");
        BRsteer = hardwareMap.get(CRServoImplEx.class, "BRsteer");

        // axon servos have a PWM range of 500us-2500us
        FRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BLsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));
        BRsteer.setPwmRange(new PwmControl.PwmRange(500, 2500));

        FRencoder = hardwareMap.get(AnalogInput.class, "FRencoder");
        FLencoder = hardwareMap.get(AnalogInput.class, "FLencoder");
        BLencoder = hardwareMap.get(AnalogInput.class, "BLencoder");
        BRencoder = hardwareMap.get(AnalogInput.class, "BRencoder");


        PIDController FRpid = new PIDController(kP, kI, kD);
        PIDController FLpid = new PIDController(kP, kI, kD);
        PIDController BLpid = new PIDController(kP, kI, kD);
        PIDController BRpid = new PIDController(kP, kI, kD);

        SlewRateLimiter fwSlew = new SlewRateLimiter(fw_r);
        SlewRateLimiter strSlew = new SlewRateLimiter(str_r);
        SlewRateLimiter rotSlew = new SlewRateLimiter(rot_r);

//        PhotonCore.experimental.setMaximumParallelCommands(8);
//        PhotonCore.enable();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double fwd = -gamepad1.left_stick_y; // Pushing joystick up is negative
            double str = gamepad1.left_stick_x; // Pushing joystick to the right is positive
            double rcw = gamepad1.right_stick_x; // Clockwise rotation is positive

            // slew rate limit
            if (slew) {
                fwd = fwSlew.calculate(fwd);
                str = strSlew.calculate(str);
                rcw = rotSlew.calculate(rcw);
            }

            // continue if no value
            if (Math.abs(fwd) < joystickLimit && Math.abs(str) < joystickLimit && Math.abs(rcw) < joystickLimit) {
                FRdrive.setPower(0);
                FLdrive.setPower(0);
                BLdrive.setPower(0);
                BRdrive.setPower(0);

                FRsteer.setPower(0);
                FLsteer.setPower(0);
                BLsteer.setPower(0);
                BRsteer.setPower(0);
                continue;
            }

            // Retrieve Rotational Angles and Velocities
//            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            // field centric
            double temp = (fwd*Math.cos(imuAngle)) + str*Math.sin(imuAngle);
            double str2 = (-fwd*Math.sin(imuAngle)) + str*Math.cos(imuAngle);
            double fwd2 = temp;

            // units doesn't matter as long as they are the same
            double wheelbase = 9.492126;
            double trackwidth = 9.492126;
            double r = Math.sqrt((wheelbase*wheelbase) + (trackwidth*trackwidth));

            double a = str2 - rcw * (wheelbase/r);
            double b = str2 + rcw * (wheelbase/r);
            double c = fwd2 - rcw * (trackwidth/r);
            double d = fwd2 + rcw * (trackwidth/r);

            // wheel speeds
            double frs = Math.sqrt(b*b + c*c);
            double fls = Math.sqrt(b*b + d*d);
            double bls = Math.sqrt(a*a + d*d);
            double brs = Math.sqrt(a*a + c*c);

            // wheel angles in degrees +- 180
            double fra = Math.atan2(b,c) * 180/Math.PI;
            double fla = Math.atan2(b,d) * 180/Math.PI;
            double bla = Math.atan2(a,d) * 180/Math.PI;
            double bra = Math.atan2(a,c) * 180/Math.PI;

            // Normalize wheel speeds
            double max = frs;
            if(fls>max){
                max = fls;
            }
            if(bls>max){
                max = bls;
            }
            if(brs>max){
                max = brs;
            }
            if(max>1){
                frs/=max;
                fls/=max;
                brs/=max;
                bls/=max;
            }


            // Set angle PIDs
            FRpid.setPID(kP, kI, kD);
            FLpid.setPID(kP, kI, kD);
            BLpid.setPID(kP, kI, kD);
            BRpid.setPID(kP, kI, kD);


            double FRpos;
            double FLpos;
            double BLpos;
            double BRpos;
            // Wheel positions
            if (FRflippedEnc) {
                FRpos = 360 - FRencoder.getVoltage() / 3.3 * 360;
            } else {
                FRpos = FRencoder.getVoltage() / 3.3 * 360;
            }

            if (FLflippedEnc) {
                FLpos = 360 - FLencoder.getVoltage() / 3.3 * 360;
            } else {
                FLpos = FLencoder.getVoltage() / 3.3 * 360;
            }

            if (BLflippedEnc) {
                BLpos = 360 - BLencoder.getVoltage() / 3.3 * 360;
            } else {
                BLpos = BLencoder.getVoltage() / 3.3 * 360;
            }

            if (BRflippedEnc) {
                BRpos = 360 - BRencoder.getVoltage() / 3.3 * 360;
            } else {
                BRpos = BRencoder.getVoltage() / 3.3 * 360;
            }
            //apply offsets
            FRpos -= FRoffset;
            FLpos -= FLoffset;
            BLpos -= BLoffset;
            BRpos -= BRoffset;



            // Errors
            double FRerror = normalizeDegrees(fra - FRpos);
            double FLerror = normalizeDegrees(fla - FLpos);
            double BLerror = normalizeDegrees(bla - BLpos);
            double BRerror = normalizeDegrees(bra - BRpos);

            // flip shortcut
            if (MOTOR_FLIPPING && Math.abs(FRerror) > 90) {
                fra = normalizeDegrees(fra - 180);
                FRflipped = true;
            } else {
                FRflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(FLerror) > 90) {
                fla = normalizeDegrees(fla - 180);
                FLflipped = true;
            } else {
                FLflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(BLerror) > 90) {
                bla = normalizeDegrees(bla - 180);
                BLflipped = true;
            } else {
                BLflipped = false;
            }
            if (MOTOR_FLIPPING && Math.abs(BRerror) > 90) {
                bra = normalizeDegrees(bra - 180);
                BRflipped = true;
            } else {
                BRflipped = false;
            }

            FRerror = normalizeDegrees(fra - FRpos);
            FLerror = normalizeDegrees(fla - FLpos);
            BLerror = normalizeDegrees(bla - BLpos);
            BRerror = normalizeDegrees(bra - BRpos);


            // Wheel PID angles
            double FRoutput = FRpid.calculate(0, FRerror);
            double FLoutput = FLpid.calculate(0, FLerror);
            double BLoutput = BLpid.calculate(0, BLerror);
            double BRoutput = BRpid.calculate(0, BRerror);

            FRsteer.setPower(FRoutput);
            FLsteer.setPower(FLoutput);
            BLsteer.setPower(BLoutput);
            BRsteer.setPower(BRoutput);





            // Set drive powers
            if (FRflipped)
                FRdrive.setPower(-frs);
            else
                FRdrive.setPower(frs);

            if (FLflipped)
                FLdrive.setPower(-fls);
            else
                FLdrive.setPower(fls);

            if (BLflipped)
                BLdrive.setPower(-bls);
            else
                BLdrive.setPower(bls);

            if (BRflipped)
                BRdrive.setPower(-brs);
            else
                BRdrive.setPower(brs);




            // telemetry wheel speeds
            telemetry.addData("FRspeed", frs);
            telemetry.addData("FLspeed", fls);
            telemetry.addData("BLspeed", bls);
            telemetry.addData("BRspeed", brs);

            // telemetry wheel angles
            telemetry.addData("FRangle", fra);
            telemetry.addData("FLangle", fla);
            telemetry.addData("BLangle", bla);
            telemetry.addData("BRangle", bra);

            // telemetry actual wheel angles
            telemetry.addData("FR Current Pos", FRpos);
            telemetry.addData("FL Current Pos", FLpos);
            telemetry.addData("BL Current Pos", BLpos);
            telemetry.addData("BR Current Pos", BRpos);



            // telemetry flipped values
            telemetry.addData("FRflipped", FRflipped);
            telemetry.addData("FLflipped", FLflipped);
            telemetry.addData("BLflipped", BLflipped);
            telemetry.addData("BRflipped", BRflipped);

            // telemetry loop times
            telemetry.addData("Loop Time: ", loopTimer.milliseconds());
            telemetry.addData("Loop Frequency: ", 1/loopTimer.milliseconds());
            loopTimer.reset();


            telemetry.update();
        }
    }
}
