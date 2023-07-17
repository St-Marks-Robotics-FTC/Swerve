package org.firstinspires.ftc.teamcode.ZeroToAuto;


import org.firstinspires.ftc.teamcode.ZeroToAuto.Constants;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class SwerveModule {

    private final MotorEx driveMotor;
    private final CRServo turningServo;

    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(HardwareMap hardwareMap, String driveMotorId, String turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        String absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = hardwareMap.get(AnalogInput.class, absoluteEncoderId);

        driveMotor = new MotorEx(hardwareMap, driveMotorId);
        turningServo = new CRServo(hardwareMap, turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningServo.setInverted(turningMotorReversed);

//        driveEncoder = driveMotor.getEncoder();
//        turningEncoder = turningServo.getEncoder();

//        driveEncoder.setPosition Conversion Factor (ModuleConstants.kDrive Encoder Rot2Meter);
//        driveEncoder.setVelocityConversionFactor (ModuleConstants.kDriveEncoder RPM2Meter PerSec);
//        turningEncoder.setPositionConversion Factor (ModuleConstants.kTurningEncoderRot2Rad);
//        turningEncoder.setVelocityConversionFactor (ModuleConstants.kTurningEncoder RPM2RadPerSec;

        turningPIDController = new PIDController(Constants.ModuleConstants.kPTurning, 0, 0);
//        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition(){
        return driveMotor.getCurrentPosition();
    }

//    public double getTurningPosition(){
//        return turningServo.getPower();
//    }

    public double getDriveVelocity(){
        return driveMotor.getVelocity();
    }
    public double getTurningVelocity(){
        return turningServo.get();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / 3.3;
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders(){
        driveMotor.resetEncoder();
//        turningEncoder.reset();
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
//        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsoluteEncoderRad()));
    }

}
