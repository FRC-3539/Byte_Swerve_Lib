package org.frcteam3539.Byte_Swerve_Lib.tests;

import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

public class TestBBMPLoader {

    public static void main(String args[]) {

        BBMPLoader loader = new BBMPLoader("C:\\Users\\camco\\Desktop\\test.txt");
        Trajectory test = loader.getNextTrajectory();

        for (double i = 0; i < test.getDuration(); i += .01) {
            System.out.println("(" + i + "," + test.calculate(i).getVelocity() + ")");
        }
    }
}
