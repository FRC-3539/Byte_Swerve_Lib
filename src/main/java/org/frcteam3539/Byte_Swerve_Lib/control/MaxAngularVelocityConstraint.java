package org.frcteam3539.Byte_Swerve_Lib.control;

public class MaxAngularVelocityConstraint extends TrajectoryConstraint {
    private final double maxAngularVelocity;

    /**
     * Creates a new MaxAngularVelocityConstraint.
     *
     * @param maxAngularVelocity the maximum angular velocity in radians per second.
     */
    public MaxAngularVelocityConstraint(double maxAngularVelocity) {
        this.maxAngularVelocity = maxAngularVelocity;
    }

    @Override
    public double getMaxVelocity(Path.State startingState, Path.State endingState) {

        double distance = endingState.getDistance() - startingState.getDistance();

        double rotationDelta = endingState.getPose2d().getRotation().minus(startingState.getPose2d().getRotation())
                .getRadians();

        double timeToRotate = Math.abs(rotationDelta) / maxAngularVelocity;

        return distance / timeToRotate;
    }
}
