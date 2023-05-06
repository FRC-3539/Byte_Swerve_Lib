package org.frcteam3539.CTRE_Swerve_Lib.control;

import org.frcteam3539.CTRE_Swerve_Lib.control.Trajectory.State;

public class MultiTrajectory {
    double duration = 0;

    Trajectory[] trajectories;
    double[] times;

    public MultiTrajectory(Trajectory... trajectories) {
        times = new double[trajectories.length];
        int i = 0;
        for (Trajectory traj : trajectories) {
            duration += traj.getDuration();
            times[i] = duration;
            i++;
        }
        this.trajectories = trajectories;
    }

    public Trajectory getTrajectory(double time) {
        if(time>duration)
        {
            return trajectories[trajectories.length-1];
        }
        for (int i = 0; i < times.length; i++) {
            if (times[i] > time) {
                return trajectories[i];
            }
        }
        return null;
    }

    public int getIndex(double time) {
        if(time>duration)
        {
            return times.length-1;
        }
        for (int i = 0; i < times.length; i++) {
            if (times[i] > time) {
                return i;
            }
        }

        return -1;
    }

    public State calculate(double time) {
        if(trajectories.length==1)
        {
            return trajectories[0].calculate(time);
        }
        Trajectory traj = getTrajectory(time);
        if (getIndex(time) != 0) {
            return traj.calculate(time-times[getIndex(time)-1]);
        }
        else {
            return traj.calculate(time);
        }

    }

    public double getDuration() {
        return duration;
    }
}
