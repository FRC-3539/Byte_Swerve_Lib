package org.frcteam3539.CTRE_Swerve_Lib.swerve;

public class SwerveDriveTrainConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    public double turnKp = 0;
    public double turnKd = 0;

    public SwerveDriveTrainConstants withPigeon2Id(int id) {
        this.pigeon2Id = id;
        return this;
    }

    public SwerveDriveTrainConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKp(double turnKp) {
        this.turnKp = turnKp;
        return this;
    }

    public SwerveDriveTrainConstants withTurnKd(double turnKd) {
        this.turnKd = turnKd;
        return this;
    }
}
