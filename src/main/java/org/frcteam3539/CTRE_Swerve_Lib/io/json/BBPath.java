// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frcteam3539.CTRE_Swerve_Lib.io.json;

/** Add your docs here. */
public abstract class BBPath {
    public abstract double[][][] getPaths();
    public abstract double[][] getConstraints();
}