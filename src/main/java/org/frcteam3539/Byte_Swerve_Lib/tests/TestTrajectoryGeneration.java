package org.frcteam3539.Byte_Swerve_Lib.tests;

import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxAngularVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestTrajectoryGeneration {

    public static void main(String args[]) {
        TrajectoryConstraint[] constraints = { new MaxVelocityConstraint(10), new MaxAccelerationConstraint(10, 10),
                new MaxAngularVelocityConstraint(1) };
        Trajectory test = new Trajectory(
                new SimplePathBuilder(new Pose2d(0, 0, new Rotation2d())) // Start at (0, 0)
                        .lineTo(new Pose2d(1, 1, Rotation2d.fromRadians(2))) // Add a valid segment
                        .build(),
                constraints, 0.01, 0, 0);

        for (double i = 0; i < test.getDuration(); i += .01) {
            System.out.println(
                    "(" + i + "," +
                            test.calculate(i).getPathState().getPose2d().getRotation().getRadians() +
                            ")");
        }
    }
}
