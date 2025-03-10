// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;
//import java.lang.Comparable;

/** Add your docs here. */
public interface InverseInterpolable<T> {

    public double inverseInterpolate(T upper, T query); 
}
