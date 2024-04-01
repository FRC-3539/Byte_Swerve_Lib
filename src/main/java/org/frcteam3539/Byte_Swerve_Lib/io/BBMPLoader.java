package org.frcteam3539.Byte_Swerve_Lib.io;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import org.frcteam3539.Byte_Swerve_Lib.control.CentripetalRadiusAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.Path;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;
import org.frcteam3539.Byte_Swerve_Lib.io.json.BBPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BBMPLoader {
    private Trajectory[] trajectories;
    private Pose2d startPose = new Pose2d();
    private int currentTrajectory = -1;

    /**
     * 
     * @param filename File name with extension but no directory.
     * @param debug    use debug prints
     */
    public BBMPLoader(String filename, boolean debug) {
        trajectories = getTrajectories(filename, debug);
    }

    /**
     * 
     * @param filename File name with extension but no directory.
     * @param debug    use debug prints
     */
    public BBMPLoader(String filename) {
        this(filename, false);
    }

    public BBMPLoader(BBPath path) {
        trajectories = getTrajectories(path.getPaths(), path.getConstraints(), false);
    }

    public BBMPLoader(BBPath path, boolean debug) {
        trajectories = getTrajectories(path.getPaths(), path.getConstraints(), debug);
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public Trajectory getFirstTrajectory() {
        if (trajectories == null) {
            return null;
        }
        if (trajectories.length == 0) {
            return null;
        }
        return trajectories[0];
    }

    /**
     * Get the next trajectory in the file.
     */
    public Trajectory getNextTrajectory() {
        if (trajectories == null) {
            return null;
        }
        if (trajectories.length == 0) {
            return null;
        }
        currentTrajectory++;
        if (currentTrajectory > trajectories.length - 1) {
            return null;
        }
        return trajectories[currentTrajectory];
    }

    /**
     * Get the Current Trajectory (must call getNextTrajectory first)
     * 
     * @return The Trajectory that was last returned from getNextTrajectory()
     */
    public Trajectory getCurrentTrajectory() {
        if (trajectories == null) {
            return null;
        }
        if (trajectories.length == 0) {
            return null;
        }
        if (currentTrajectory > trajectories.length - 1) {
            return null;
        }
        return trajectories[currentTrajectory];
    }

    public Trajectory[] getTrajectories() {
        return trajectories;
    }

    public int getTrajectoryCount() {
        return trajectories.length;
    }

    private Trajectory[] getTrajectories(String filename, boolean debug) {

        if (debug)
            System.out.println("\n\n\n\nfilename: " + filename + "\n\nlines:");

        ArrayList<ArrayList<String>> paths = new ArrayList<ArrayList<String>>();
        ArrayList<String> path = new ArrayList<String>();
        ArrayList<String> constraints = new ArrayList<String>();

        try {
            BufferedReader reader = new BufferedReader(new FileReader(filename));
            String line;
            while ((line = reader.readLine()) != null) {
                if (line.startsWith("@")) {
                    // End of path detected
                    paths.add(path);
                    path = new ArrayList<String>();
                } else if (paths.size() == constraints.size()) {
                    // Add path constraints
                    constraints.add(line);
                } else {
                    path.add(line);
                }
            }
            reader.close();
        } catch (Exception e) {
        }

        double[][][] pathsParsed = new double[paths.size()][][];
        for (int i = 0; i < paths.size(); i++) {
            double[][] pathParsed = new double[paths.get(i).size()][];
            for (int j = 0; j < paths.get(i).size(); j++) {
                String points = paths.get(i).get(j);
                String[] pointData = points.split(" ");
                double[] parsedPoints = new double[4];
                for (int k = 0; k < 4; k++) {
                    parsedPoints[k] = Double.parseDouble(pointData[k]);
                }
                pathParsed[j] = parsedPoints;
            }
            pathsParsed[i] = pathParsed;
        }

        double[][] constraintsParsed = new double[paths.size()][];
        for (int i = 0; i < paths.size(); i++) {
            String[] constraintsList = constraints.get(i).split(" ");
            if (constraintsList.length == 3) {
                constraintsParsed[i] = new double[] {
                        Double.parseDouble(constraintsList[0]),
                        Double.parseDouble(constraintsList[1]),
                        Double.parseDouble(constraintsList[2]),
                        0,
                        0
                };
            } else if (constraintsList.length == 5) {
                constraintsParsed[i] = new double[] {
                        Double.parseDouble(constraintsList[0]),
                        Double.parseDouble(constraintsList[1]),
                        Double.parseDouble(constraintsList[2]),
                        Double.parseDouble(constraintsList[3]),
                        Double.parseDouble(constraintsList[4])
                };
            } else {
                System.out.println("PATH PARSE ERROR");
            }

        }

        return getTrajectories(pathsParsed, constraintsParsed, debug);
    }

    // Fill a command sequence from a pre-compiled or parsed double array
    // (recommended for comp)
    public Trajectory[] getTrajectories(double[][][] paths, double[][] constraints, boolean debug) {

        if (debug)
            System.out.println("received:");
        if (debug)
            for (double[][] ds : paths) {
                for (double[] ds2 : ds) {
                    for (double ds3 : ds2) {
                        System.out.print(ds3 + " ");
                    }
                    System.out.println();
                }
            }

        Trajectory[] sequence = new Trajectory[paths.length];

        if (paths.length == 0) {
            return new Trajectory[0];
        }

        startPose = new Pose2d(
                paths[0][0][0],
                paths[0][0][1],
                Rotation2d.fromDegrees(paths[0][0][2]));

        if (debug)
            System.out.println("\n\n\n");

        for (int pathIdx = 0; pathIdx < paths.length; pathIdx++) {
            SimplePathBuilder builder = new SimplePathBuilder(
                    new Pose2d(paths[pathIdx][0][0], paths[pathIdx][0][1],
                            Rotation2d.fromDegrees(paths[pathIdx][0][2]))

            );

            for (int point = 1; point < paths[pathIdx].length; point++) {
                builder.lineWithRadiusTo(new Pose2d(paths[pathIdx][point][0], paths[pathIdx][point][1],
                        Rotation2d.fromDegrees(paths[pathIdx][point][2])), paths[pathIdx][point][3]);

            }

            Path p = builder.build();
            Trajectory trajectory = new Trajectory(p,
                    getConstraints(constraints[pathIdx][0], constraints[pathIdx][1], constraints[pathIdx][2]),
                    0.02, constraints[pathIdx][3], constraints[pathIdx][4]);

            sequence[pathIdx] = trajectory;
        }
        return sequence;
    }

    public TrajectoryConstraint[] getConstraints(double velocity, double acceleration,
            double centripetalAcceleration) {
        return new TrajectoryConstraint[] {
                (TrajectoryConstraint) new MaxAccelerationConstraint(acceleration),
                (TrajectoryConstraint) new MaxVelocityConstraint(velocity),
                (TrajectoryConstraint) new CentripetalRadiusAccelerationConstraint(centripetalAcceleration)
        };
    }
}