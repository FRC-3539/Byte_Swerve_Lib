package org.frcteam3539.CTRE_Swerve_Lib.control;

import org.frcteam3539.CTRE_Swerve_Lib.util.HolonomicDriveSignal;
import org.frcteam3539.CTRE_Swerve_Lib.util.HolonomicFeedforward;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
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
        this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
                                                        double rotationalVelocity, Trajectory trajectory, double time,
                                                        double dt) {
        if (time > trajectory.getDuration()) {
            finished = true;
            return new HolonomicDriveSignal(new Translation2d(), 0.0, false);
        }

        lastState = trajectory.calculate(time);

        Translation2d segment =  new Translation2d(
                lastState.getPathState().getHeading().getCos(),
                lastState.getPathState().getHeading().getSin());

        Translation2d segmentVelocity = segment.times(lastState.getVelocity());
        Translation2d segmentAcceleration = segment.times(lastState.getAcceleration());

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        forwardController.setSetpoint(lastState.getPathState().getPosition().getX());
        strafeController.setSetpoint(lastState.getPathState().getPosition().getY());
        rotationController.setSetpoint(lastState.getPathState().getRotation().getRadians());

        return new HolonomicDriveSignal(
                new Translation2d(
                        forwardController.calculate(currentPose.getTranslation().getX(), dt) + feedforwardVector.getX(),
                        strafeController.calculate(currentPose.getTranslation().getY(), dt) + feedforwardVector.getY()
                ),
                rotationController.calculate(currentPose.getRotation().getRadians(), dt),
                true
        );
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
