package com.drone.sim;

public class Path3D {
    public CubicHermite3D spline;
    public float maxVel, maxAccel, maxDecel, endVel;

    public Path3D(CubicHermite3D spline, float maxVel, float maxAccel, float maxDecel, float endVel) {
        this.spline = spline;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.endVel = endVel;
    }

    public Path3D(CubicHermite3D path, float maxVel, float maxAccel, float maxDecel) {
        this(path, maxVel, maxAccel, maxDecel, 0);
    }
}
