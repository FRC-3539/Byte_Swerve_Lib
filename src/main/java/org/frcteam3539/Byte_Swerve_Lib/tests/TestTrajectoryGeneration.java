package org.frcteam3539.Byte_Swerve_Lib.tests;

import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TestTrajectoryGeneration {

    public static void main(String args[]) {
        TrajectoryConstraint[] constraints = { new MaxVelocityConstraint(2), new MaxAccelerationConstraint(2) };
        Trajectory test = new Trajectory(
                new SimplePathBuilder(new Pose2d()).lineTo(new Pose2d(1, 1, Rotation2d.fromDegrees(0))).build(),
                constraints, .01, 0,
                0);

        for (double i = 0; i < test.getDuration(); i += .01) {
            System.out.println("(" + i + "," + test.calculate(i).getVelocity() + ")");
        }
    }
}
