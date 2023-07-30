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

    /**
     * Class to help create multiple swerve module constants
     * @param swerveModuleDriveRatio Drive ratio for swerve modules. 10:1 would be 10.0
     * @param swerveModuleSteerRatio Steer ratio for swerve modules. 10:1 would be 10.0
     * @param swerveModuleWheelRadius Radius of swerve module wheel (recommended to be in meters).
     * @param swerveModuleSlipCurrent Module slip current (limits motor torque to prevent wheel slip)
     * @param swerveModuleSteerGains Module steering gains aka pid.
     * @param swerveModuleDriveGains Module drive gains aka pid.
     * @param steerMotorReversed is the motor steering inverted from the cancoder.
     */
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

    /**
     * 
     * @param steerId Steering id for specific module.
     * @param driveId Drive id for specific module.
     * @param cancoderId CanCoder id for specific module.
     * @param cancoderOffset CanCoder offset for specific module in rotations.
     * @param locationX X Location of the swerve module. Uses WPILIB cordinate system.
     * @param locationY Y Location of the swerve module. Uses WPILIB cordinate system.
     * @return the specific module constants.
     */
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
