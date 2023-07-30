package org.frcteam3539.CTRE_Swerve_Lib.swerve;

public class SwerveDriveTrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    public double turnKp = 0;
    public double turnKd = 0;

    /**
     * 
     * @param id of the pigeon2
     * @return Swerve Drivetrain Constants
     */
    public SwerveDriveTrainConstants withPigeon2Id(int id) {
        this.pigeon2Id = id;
        return this;
    }

    /**
     * 
     * @param name of the canbus
     * @return Swerve Drivetrain Constants
     */
    public SwerveDriveTrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }
    /**
     * 
     * @param turnKp P gain for robot turning.
     * @return Swerve Drivetrain Constants
     */
    public SwerveDriveTrainConstants withTurnKp(double turnKp) {
        this.turnKp = turnKp;
        return this;
    }
    /**
     * 
     * @param turnKd D gain for robot turning.
     * @return Swerve Drivetrain Constants
     */
    public SwerveDriveTrainConstants withTurnKd(double turnKd) {
        this.turnKd = turnKd;
        return this;
    }
}
