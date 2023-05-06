
package org.frcteam3539.CTRE_Swerve_Lib.util;

public interface InverseInterpolatable<T> {
    double inverseInterpolate(T upper, T query);
}
