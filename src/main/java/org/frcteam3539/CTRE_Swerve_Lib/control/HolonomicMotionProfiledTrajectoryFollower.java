package org.frcteam3539.CTRE_Swerve_Lib.control;

import org.frcteam3539.CTRE_Swerve_Lib.util.HolonomicFeedforward;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<ChassisSpeeds> {
    private PidController forwardController;
    private PidController strafeController;
    private PidController rotationController;

    private HolonomicFeedforward feedforward;

    private Trajectory.State lastState = null;

    private boolean finished = false;

    public HolonomicMotionProfiledTrajectoryFollower(PidConstants translationConstants, PidConstants rotationConstants,
            HolonomicFeedforward feedforward) {
        this.forwardController = new PidController(translationConstants);
        this.strafeController = new PidController(translationConstants);
        this.rotationController = new PidController(rotationConstants);
        this.rotationController.setContinuous(true);
        this.rotationController.setInputRange(-Math.PI, Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected ChassisSpeeds calculateDriveSignal(Pose2d currentPose, Trajectory trajectory, double time, double dt) {
        if (time > trajectory.getDuration()) {
            finished = true;
            return ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,Rotation2d.fromDegrees(0));
        }

        lastState = trajectory.calculate(time);

        Translation2d segment = new Translation2d(
                lastState.getPathState().getHeading().getCos(),
                lastState.getPathState().getHeading().getSin());

        Translation2d segmentVelocity = segment.times(lastState.getVelocity());
        Translation2d segmentAcceleration = segment.times(lastState.getAcceleration());

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(lastState.getPathState().getPose2d().getX());
        strafeController.setSetpoint(lastState.getPathState().getPose2d().getY());
        rotationController.setSetpoint(lastState.getPathState().getPose2d().getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardController.calculate(currentPose.getTranslation().getX(), dt) + feedforwardVector.getX(),
                strafeController.calculate(currentPose.getTranslation().getY(), dt) + feedforwardVector.getY(),
                rotationController.calculate(currentPose.getRotation().getRadians(), dt), currentPose.getRotation());
    }

    public Trajectory.State getLastState() {
        return lastState;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();

        finished = false;
    }
}
