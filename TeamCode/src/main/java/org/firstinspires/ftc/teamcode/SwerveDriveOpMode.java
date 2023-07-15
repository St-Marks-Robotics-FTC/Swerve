package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class SwerveDriveOpMode extends LinearOpMode {
    private DcMotor FLdrive, FRdrive, BLdrive, BRdrive;
    private Servo FLsteer, FRsteer, BLsteer, BRsteer;
    private AnalogInput FLencoder, FRencoder, BLencoder, BRencoder;

    // The IMU sensor object
    IMU imu;


    private double[] drivePowers;
    private double[] steerPositions;


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

        FLsteer = hardwareMap.get(Servo.class, "FLsteer");
        FRsteer = hardwareMap.get(Servo.class, "FRsteer");
        BLsteer = hardwareMap.get(Servo.class, "BLsteer");
        BRsteer = hardwareMap.get(Servo.class, "BRsteer");

        FLencoder = hardwareMap.get(AnalogInput.class, "FLencoder");
        FRencoder = hardwareMap.get(AnalogInput.class, "FRencoder");
        BLencoder = hardwareMap.get(AnalogInput.class, "BLencoder");
        BRencoder = hardwareMap.get(AnalogInput.class, "BRencoder");



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

            double wheelbase = 30.0; // inches
            double trackwidth = 24.0; // inches
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

            // wheel angles
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





            telemetry.update();
        }
    }
}
