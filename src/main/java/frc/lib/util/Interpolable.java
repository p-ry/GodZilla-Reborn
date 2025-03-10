// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;
//import java.lang.Comparable;
/** Add your docs here. */
public interface Interpolable<T> {

    

      /**
     * Interpolates between this value and an other value according to a given
     * parameter. If x is 0, the method should return this value. If x is 1, the
     * method should return the other value. If 0 < x < 1, the return value
     * should be interpolated proportionally between the two.
     *
     * @param other
     *            The value of the upper bound
     * @param x
     *            The requested value. Should be between 0 and 1.
     * @return Interpolable<T> The estimated average between the surrounding
     *         data
     */
    public T interpolate(T other, double x);
}

