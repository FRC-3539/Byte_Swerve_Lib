package org.frcteam3539.Byte_Swerve_Lib.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class PathSegment {
    public State getStart() {
        return calculate(0.0);
    }

    public State getEnd() {
        return calculate(getLength());
    }

    public abstract State calculate(double distance);

    public abstract double getLength();

    public abstract double getRadius();

    public static class State {
        private final Translation2d position;
        private final Rotation2d heading;
        private final double curvature;

        public State(Translation2d position, Rotation2d heading, double curvature) {
            this.position = position;
            this.heading = heading;
            this.curvature = curvature;
        }

        /**
         * 
         * @return The location of the robot.
         */
        public Translation2d getPosition() {
            return position;
        }

        /**
         * 
         * @return The heading that the robot is following. This is not the direction/angle the robot is facing this is the direction it is moving.
         */
        public Rotation2d getHeading() {
            return heading;
        }

        /**
         * 
         * @return The curvature of the path.
         */
        public double getCurvature() {
            return curvature;
        }
    }
}
