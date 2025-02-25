package org.firstinspires.ftc.teamcode.ZeroToAuto;

import com.arcrobotics.ftclib.util.MathUtils;

public class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 4 *.0254;
        public static final double kDriveMotorGearRatio= 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio* Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;

    }
}
