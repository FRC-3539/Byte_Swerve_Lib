package org.frcteam3539.CTRE_Swerve_Lib.math.spline;

import edu.wpi.first.math.geometry.Translation2d;

import static org.frcteam3539.CTRE_Swerve_Lib.math.spline.SplineHelper.binomial;

import org.ejml.simple.SimpleMatrix;

class BezierSplineHelper {
    private BezierSplineHelper() {
    }

    public static SimpleMatrix createBasisMatrix(int degree) {
        SimpleMatrix matrix = new SimpleMatrix(degree + 1, degree + 1);
        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j <= i; j++) {
                matrix.set(i, j, Math.pow(-1, i - j) * binomial(degree, i) * binomial(i, j));
            }
        }

        return matrix;
    }

    public static SimpleMatrix createBasisWeightMatrix(Translation2d... controlPoints) {
        SimpleMatrix basisWeightMatrix = new SimpleMatrix(controlPoints.length, 2);
        for (int i = 0; i < controlPoints.length; i++) {
            basisWeightMatrix.setRow(i, 0, controlPoints[i].getX(), controlPoints[i].getY());
        }
        return basisWeightMatrix;
    }

    public static Translation2d[] basisWeightMatrixToControlPoints(SimpleMatrix basisWeightMatrix) {
        if (basisWeightMatrix.numCols() != 2) {
            throw new IllegalArgumentException("Basis weight matrix must have 2 columns");
        }

        Translation2d[] controlPoints = new Translation2d[basisWeightMatrix.numRows()];
        for (int i = 0; i < controlPoints.length; i++) {
            controlPoints[i] = new Translation2d(basisWeightMatrix.get(i, 0), basisWeightMatrix.get(i, 1));
        }
        return controlPoints;
    }
}
