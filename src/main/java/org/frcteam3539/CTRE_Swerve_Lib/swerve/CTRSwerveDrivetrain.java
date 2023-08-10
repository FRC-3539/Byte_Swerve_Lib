package org.frcteam3539.CTRE_Swerve_Lib.swerve;

import java.util.Arrays;
import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


/**
 * A Swerve class that takes advantage of the benifits of pheonix pro for more accurate control of your robot.
 */
public class CTRSwerveDrivetrain {
    private final int ModuleCount;

    private CTRSwerveModule[] m_modules;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    public SwerveDrivePoseEstimator m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Field2d m_field;
    private PIDController m_turnPid;
    private double MAX_VELOCITY_METERS_PER_SECOND = Double.MAX_VALUE;
    private double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 0.0;

    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignalValue[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;

        public OdometryThread(ShuffleboardTab tab) {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignalValue[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = m_modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZ();

            tab.addNumber("Successful Daqs", ()->{return SuccessfulDaqs;});
            tab.addNumber("Failed Daqs", ()->{return FailedDaqs;});
            tab.addNumber("X Pos", ()->{return m_odometry.getEstimatedPosition().getX();});
            tab.addNumber("Y Pos", ()->{return m_odometry.getEstimatedPosition().getY();});
            tab.addNumber("Angle", ()->{return m_odometry.getEstimatedPosition().getRotation().getDegrees();});
        }
        @Override
        public void run() {
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                BaseStatusSignalValue.waitForAll(0.1, m_allSignals);

                /* Get status of first element */
                if (m_allSignals[0].getError().isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < ModuleCount; ++i) {
                    m_modulePositions[i] = m_modules[i].getPosition();
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees =
                        BaseStatusSignalValue.getLatencyCompensatedValue(
                                m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZ());
                synchronized(m_odometry)
                {
                    m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
                }
                m_field.setRobotPose(m_odometry.getEstimatedPosition());
            }
        }
    }
    /**
     * 
     * @param tab Shuffleboard tab, Pose2d, successes, and errors will be linked to this.
     * @param driveTrainConstants Swerve drive constants including canbus name, robot rotation pid values, and pigeon id.
     * @param modules All the swervemodule constants including motor ids, cancoder ids, encoder offsets and other module constants.
     * @see ShuffleboardTab
     * @see SwerveDriveTrainConstants
     * @see SwerveModuleConstants
     */
    public CTRSwerveDrivetrain(ShuffleboardTab tab,
            SwerveDriveTrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        ModuleCount = modules.length;

        m_pigeon2 = new Pigeon2(driveTrainConstants.pigeon2Id, driveTrainConstants.CANbusName);

        m_modules = new CTRSwerveModule[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];


        int iteration = 0;
        for (SwerveModuleConstants module : modules) {
            double maxModuleVel = (2 * Math.PI * module.WheelRadius)*((6080/60.0)/ module.driveMotorGearRatio);//circumference*(max motor rps) = max module velocity;
            if(maxModuleVel<MAX_VELOCITY_METERS_PER_SECOND)
            {
                MAX_VELOCITY_METERS_PER_SECOND = maxModuleVel;
            }
            
            m_modules[iteration] = new CTRSwerveModule(module, driveTrainConstants.CANbusName);
            m_moduleLocations[iteration] = new Translation2d(module.locationX, module.locationY);
            m_modulePositions[iteration] = m_modules[iteration].getPosition();

            iteration++;
        }
        
        double dtRadius = new Translation2d().nearest(Arrays.asList(m_moduleLocations)).getDistance(new Translation2d());
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND/ dtRadius;


        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        
        m_odometry = new SwerveDrivePoseEstimator(
			m_kinematics,
			getGyroAngle(),
			getSwervePositions(),
			new Pose2d(0, 0, new Rotation2d()),
			VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0)),
			VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10))
		);
        m_field = new Field2d();
        tab.add("Field", m_field);

        m_turnPid = new PIDController(driveTrainConstants.turnKp, 0, driveTrainConstants.turnKd);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);

        m_odometryThread = new OdometryThread(tab);
        m_odometryThread.start();
    }

    /**
     * 
     * @return The list of module positions. Given in the order as which this was constructed.
     */
    public SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    /**
     * 
     * @return The list of modules. Given in the order as which this was constructed.
     */
    public CTRSwerveModule[] getModules() {
        return m_modules;
    }

    /**
     * Drive the robot using chassis speeds. 
     * @param speeds 
     * @see ChassisSpeeds
     */
    public void driveRobotCentric(ChassisSpeeds speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    /**
     * Reset the robots position.
     * @param pose2d pose to set the odometry to.
     * @see Pose2d
     */
    public void resetPosition(Pose2d pose2d)
    {
        m_odometry.resetPosition(getGyroAngle(), m_modulePositions, pose2d);
    }


    private Rotation2d getGyroAngle()
    {
        return Rotation2d.fromDegrees(BaseStatusSignalValue.getLatencyCompensatedValue(
                                m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZ()));
    }

    /**
     * Use full for getting angles from the pigeon for use in other subsystems and commands.
     * @return
     */
    public Pigeon2 getPigeon2()
    {
        return this.m_pigeon2;
    }

    /**
     * Drive using xy speeds and an angle which the robot will turn to face.
     * @param xSpeeds
     * @param ySpeeds
     * @param targetAngle
     */
    public void driveFullyFieldCentric(double xSpeeds, double ySpeeds, Rotation2d targetAngle) {
        var currentAngle = getGyroAngle();
        double rotationalSpeed =
                m_turnPid.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        var roboCentric =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeeds, ySpeeds, rotationalSpeed, getGyroAngle());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY_METERS_PER_SECOND);
        
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    /**
     * Get max speed of the robot. Useful for scaling joysticks to chassis speeds.
     * @return
     */
    public double getMaxVelocity()
    {
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    /**
     * Get max angular speed of the robot. Useful for scaling joysticks to chassis speeds.
     * @return
     */
    public double getMaxRotationVelocity()
    {
        return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    /**
     * Stop robot motion and lock motors in an x pattern.
     */
    public void driveStopMotion() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < ModuleCount; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].apply(new SwerveModuleState(0, angle));
        }
    }

    /**
     * Set pigeon angle to 0.
     */
    public void seedFieldRelative() {
        m_pigeon2.setYaw(0);
    }

    /**
     * Set pigeon angle.
     * @param angle
     */
    public void setGyro(double angle) {
        m_pigeon2.setYaw(angle);
    }

    /**
     * Get robot pose.
     * @return Robot pose
     * @see Pose2d
     */
    public Pose2d getPoseMeters() {
        return m_odometry.getEstimatedPosition();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }
}