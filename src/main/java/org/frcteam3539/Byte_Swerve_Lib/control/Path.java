package org.frcteam3539.Byte_Swerve_Lib.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.text.DecimalFormat;
import java.util.Map;

import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder.LineSegmentWithRadius;
import org.frcteam3539.Byte_Swerve_Lib.util.InterpolatingDouble;
import org.frcteam3539.Byte_Swerve_Lib.util.InterpolatingTreeMap;

public class Path {
    private final PathSegment[] segments;
    private final InterpolatingTreeMap<InterpolatingDouble, Rotation2d> rotationMap = new InterpolatingTreeMap<>();
    private final double[] distancesFromStart;

    private final double length;

    public Path(PathSegment[] segments, Map<Double, Rotation2d> rotationMap) {
        this.segments = segments;

        for (Map.Entry<Double, Rotation2d> rotationEntry : rotationMap.entrySet()) {
            this.rotationMap.put(new InterpolatingDouble(rotationEntry.getKey()), rotationEntry.getValue());
        }

        

        distancesFromStart = new double[segments.length];
        double cumulativeLength = 0.0;
        for (int i = 0; i < segments.length; i++) {
            distancesFromStart[i] = cumulativeLength;
            cumulativeLength += segments[i].getLength();
        }
        this.length = cumulativeLength;
    }

    private double getDistanceToSegmentStart(int segment) {
        return distancesFromStart[segment];
    }

    private double getDistanceToSegmentEnd(int segment) {
        return distancesFromStart[segment] + segments[segment].getLength();
    }

    private int getSegmentAtDistance(double distance) {
        int start = 0;
        int end = segments.length - 1;
        int mid = start + (end - start) / 2;

        while (start <= end) {
            mid = (start + end) / 2;

            if (distance > getDistanceToSegmentEnd(mid)) {
                start = mid + 1;
            } else if (distance < getDistanceToSegmentStart(mid)) {
                end = mid - 1;
            } else {
                break;
            }
        }

        return mid;
    }

    public State calculate(double distance) {
        int currentSegment = getSegmentAtDistance(distance);
        PathSegment segment = segments[currentSegment];
        double segmentDistance = distance - getDistanceToSegmentStart(currentSegment);

        PathSegment.State state = segment.calculate(segmentDistance);


        return new Path.State(
                distance,
                new Pose2d(state.getPosition(), rotationMap.getInterpolated(new InterpolatingDouble(distance))),
                state.getHeading(),
                state.getCurvature(), segment.getRadius());
    }

    public double getLength() {
        return length;
    }

    public PathSegment[] getSegments() {
        return segments;
    }

    public InterpolatingTreeMap<InterpolatingDouble, Rotation2d> getRotationMap() {
        return rotationMap;
    }

    public static class State {
        private final double distance;
        private final Pose2d pose;
        private double radius = 0.0;
        private final Rotation2d heading;
        private final double curvature;

        public State(double distance, Pose2d pose, Rotation2d heading, double curvature) {
            this.distance = distance;
            this.pose = pose;
            this.heading = heading;
            this.curvature = curvature;
        }

        public State(double distance, Pose2d pose, Rotation2d heading, double curvature, double radius) {
            this.distance = distance;
            this.pose = pose;
            this.heading = heading;
            this.curvature = curvature;
            this.radius = radius;
        }

        public double getRadius()
        {
            return radius;
        }

        public double getDistance() {
            return distance;
        }

        public Pose2d getPose2d() {
            return pose;
        }

        public Rotation2d getHeading() {
            return heading;
        }

        public double getCurvature() {
            return curvature;
        }

        @Override
        public String toString() {
            final DecimalFormat fmt = new DecimalFormat("#0.000");
            return "(distance," + fmt.format(getDistance()) +
                    ",pose," + getPose2d() +
                    ",heading," + getHeading() +
                    ",curvature," + fmt.format(getCurvature()) + " )";
        }

    }
}
