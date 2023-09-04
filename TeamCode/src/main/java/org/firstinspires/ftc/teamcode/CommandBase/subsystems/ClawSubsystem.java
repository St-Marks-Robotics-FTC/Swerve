package org.firstinspires.ftc.teamcode.CommandBase.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A gripper mechanism that grabs a stone from the quarry.
 * Centered around the Skystone game for FTC that was done in the 2019
 * to 2020 season.
 */
public class ClawSubsystem extends SubsystemBase {

    private final Servo mechRotation;

    public ClawSubsystem(final HardwareMap hMap, final String name) {
        mechRotation = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        mechRotation.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        mechRotation.setPosition(0);
    }

}