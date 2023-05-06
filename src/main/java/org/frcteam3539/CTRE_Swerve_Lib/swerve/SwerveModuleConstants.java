package org.frcteam3539.CTRE_Swerve_Lib.swerve;

import com.ctre.phoenixpro.configs.Slot0Configs;

public class SwerveModuleConstants {
    /** CAN ID of the drive motor */
    public int driveMotorId = 0;
    /** CAN ID of the steer motor */
    public int steerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth */
    public int CANcoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double CANcoderOffset = 0;
    /** Gear ratio between drive motor and wheel */
    public double driveMotorGearRatio = 0;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double steerMotorGearRatio = 0;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot
     */
    public double locationX = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot
     */
    public double locationY = 0;

    /** The steer motor gains */
    public Slot0Configs steerMotorGains = new Slot0Configs();
    /** The drive motor gains */
    public Slot0Configs driveMotorGains = new Slot0Configs();

    /** The maximum amount of current the drive motors can apply without slippage */
    public double slipCurrent = 400;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean steerMotorReversed = false;

    public SwerveModuleConstants withDriveMotorId(int id) {
        this.driveMotorId = id;
        return this;
    }

    public SwerveModuleConstants withSteerMotorId(int id) {
        this.steerMotorId = id;
        return this;
    }

    public SwerveModuleConstants withCANcoderId(int id) {
        this.CANcoderId = id;
        return this;
    }

    public SwerveModuleConstants withCANcoderOffset(double offset) {
        this.CANcoderOffset = offset;
        return this;
    }

    public SwerveModuleConstants withDriveMotorGearRatio(double ratio) {
        this.driveMotorGearRatio = ratio;
        return this;
    }

    public SwerveModuleConstants withSteerMotorGearRatio(double ratio) {
        this.steerMotorGearRatio = ratio;
        return this;
    }

    public SwerveModuleConstants withWheelRadius(double radius) {
        this.WheelRadius = radius;
        return this;
    }

    public SwerveModuleConstants withLocationX(double locationXMeters) {
        this.locationX = locationXMeters;
        return this;
    }

    public SwerveModuleConstants withLocationY(double locationYMeters) {
        this.locationY = locationYMeters;
        return this;
    }

    public SwerveModuleConstants withSteerMotorGains(Slot0Configs gains) {
        this.steerMotorGains = gains;
        return this;
    }

    public SwerveModuleConstants withDriveMotorGains(Slot0Configs gains) {
        this.driveMotorGains = gains;
        return this;
    }

    public SwerveModuleConstants withSlipCurrent(double slipCurrent) {
        this.slipCurrent = slipCurrent;
        return this;
    }

    public SwerveModuleConstants withSteerMotorReversed(boolean steerMotorReversed) {
        this.steerMotorReversed = steerMotorReversed;
        return this;
    }
}
