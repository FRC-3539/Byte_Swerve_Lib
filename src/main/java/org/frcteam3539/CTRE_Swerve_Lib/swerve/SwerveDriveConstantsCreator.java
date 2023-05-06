package org.frcteam3539.CTRE_Swerve_Lib.swerve;

import com.ctre.phoenixpro.configs.Slot0Configs;

public class SwerveDriveConstantsCreator {
    /** Gear ratio between drive motor and wheel */
    public double driveMotorGearRatio;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double steerMotorGearRatio;
    /** Wheel radius of the driving wheel in inches */
    public double wheelRadius;
    /** The maximum amount of current the drive motors can apply without slippage */
    public double slipCurrent = 400;

    /** The steer motor gains */
    public Slot0Configs steerMotorGains;
    /** The drive motor gains */
    public Slot0Configs driveMotorGains;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean steerMotorReversed;

    public SwerveDriveConstantsCreator(
            double swerveModuleDriveRatio,
            double swerveModuleSteerRatio,
            double swerveModuleWheelRadius,
            double swerveModuleSlipCurrent,
            Slot0Configs swerveModuleSteerGains,
            Slot0Configs swerveModuleDriveGains,
            boolean steerMotorReversed) {
        this.driveMotorGearRatio = swerveModuleDriveRatio;
        this.steerMotorGearRatio = swerveModuleSteerRatio;
        this.wheelRadius = swerveModuleWheelRadius;
        this.slipCurrent = swerveModuleSlipCurrent;

        this.steerMotorGains = swerveModuleSteerGains;
        this.driveMotorGains = swerveModuleDriveGains;
        this.steerMotorReversed = steerMotorReversed;
    }

    public SwerveModuleConstants createModuleConstants(
            int steerId,
            int driveId,
            int cancoderId,
            double cancoderOffset,
            double locationX,
            double locationY) {
        return new SwerveModuleConstants()
                .withSteerMotorId(steerId)
                .withDriveMotorId(driveId)
                .withCANcoderId(cancoderId)
                .withCANcoderOffset(cancoderOffset)
                .withLocationX(locationX)
                .withLocationY(locationY)
                .withDriveMotorGearRatio(driveMotorGearRatio)
                .withSteerMotorGearRatio(steerMotorGearRatio)
                .withWheelRadius(wheelRadius)
                .withSlipCurrent(slipCurrent)
                .withSteerMotorGains(steerMotorGains)
                .withDriveMotorGains(driveMotorGains)
                .withSteerMotorReversed(steerMotorReversed);
    }
}
