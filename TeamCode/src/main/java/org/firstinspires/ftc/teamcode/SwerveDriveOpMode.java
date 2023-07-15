package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SwerveDriveOpMode extends LinearOpMode {
    private DcMotor[] driveMotors;
    private Servo[] steerServos;

    private double[] drivePowers;
    private double[] steerPositions;

    @Override
    public void runOpMode() {
        // Initialize hardware components
        driveMotors = new DcMotor[] {
                hardwareMap.get(DcMotor.class, "driveMotor1"),
                hardwareMap.get(DcMotor.class, "driveMotor2"),
                hardwareMap.get(DcMotor.class, "driveMotor3"),
                hardwareMap.get(DcMotor.class, "driveMotor4")
        };

        steerServos = new Servo[] {
                hardwareMap.get(Servo.class, "steerServo1"),
                hardwareMap.get(Servo.class, "steerServo2"),
                hardwareMap.get(Servo.class, "steerServo3"),
                hardwareMap.get(Servo.class, "steerServo4")
        };

        // Initialize drive powers and steer positions
        drivePowers = new double[4];
        steerPositions = new double[4];

        // Set initial powers and positions
        for (int i = 0; i < 4; i++) {
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drivePowers[i] = 0.0;
            steerPositions[i] = 0.5;
        }

        waitForStart();

        while (opModeIsActive()) {
            // Update drive powers and steer positions based on gamepad input
            updateDrivePowers(gamepad1);
            updateSteerPositions(gamepad1);

            // Apply drive powers and steer positions to motors and servos
            for (int i = 0; i < 4; i++) {
                driveMotors[i].setPower(drivePowers[i]);
                steerServos[i].setPosition(steerPositions[i]);
            }

            telemetry.update();
            idle();
        }
    }

    private void updateDrivePowers(Gamepad gamepad) {
        double driveX = -gamepad.left_stick_x; // Negative because of the coordinate system
        double driveY = -gamepad.left_stick_y; // Negative because of the coordinate system
        double rotate = gamepad.right_stick_x;

        // Calculate individual drive motor powers
        double[] motorPowers = new double[4];
        motorPowers[0] = driveY + driveX + rotate;
        motorPowers[1] = driveY - driveX - rotate;
        motorPowers[2] = driveY - driveX + rotate;
        motorPowers[3] = driveY + driveX - rotate;

        // Normalize the motor powers to keep them within the range of -1 to 1
        double maxPower = Math.max(Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1])),
                Math.max(Math.abs(motorPowers[2]), Math.abs(motorPowers[3])));

        if (maxPower > 1.0) {
            for (int i = 0; i < 4; i++) {
                motorPowers[i] /= maxPower;
            }
        }

        // Apply exponential scaling to the drive motor powers for smoother control
        double exponent = 2.0;
        for (int i = 0; i < 4; i++) {
            drivePowers[i] = Math.copySign(Math.pow(Math.abs(motorPowers[i]), exponent), motorPowers[i]);
        }
    }

    private void updateSteerPositions(Gamepad gamepad) {
        steerPositions[0] = Range.scale(gamepad.left_trigger, 0.0, 1.0, 0.0, 1.0);
        steerPositions[1] = Range.scale(gamepad.right_trigger, 0.0, 1.0, 0.0, 1.0);
        steerPositions[2] = Range.scale(gamepad.left_trigger, 0.0, 1.0, 0.0, 1.0);
        steerPositions[3] = Range.scale(gamepad.right_trigger, 0.0, 1.0, 0.0, 1.0);
    }
}
