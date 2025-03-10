// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/** Add your docs here. */
public class MovingAvg {
    private int size;
    private double total;
    private int index;
    private double samples[];
    private double scale;
    public MovingAvg(int size){
        this.size=size;
        samples = new double[size];
        total=0;
        index=0;
       

    }
    public void add(double x){
        total -=samples[index];
        samples[index] = x;
        total += x;
        if (++index==size) index=0;
    }
    public double getAverage(){
        return total/size;
    }
}
