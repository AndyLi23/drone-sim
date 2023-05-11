package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Path3D {
    public CubicHermite3D spline;
    public float maxVel, maxAccel, maxDecel, endVel, lookahead;
    public Vector3 closest, prevClosest;
    public Vector3 lookaheadPoint, prevLookaheadPoint;
    public Kinematics kinematics;


    public Path3D(CubicHermite3D spline, float lookahead, float maxVel, float maxAccel, float maxDecel, float endVel) {
        this.spline = spline;
        this.lookahead = lookahead;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.endVel = endVel;
    }

    public Path3D(CubicHermite3D path, float lookahead, float maxVel, float maxAccel, float maxDecel) {
        this(path, lookahead, maxVel, maxAccel, maxDecel, 0);
    }

    public void init(Kinematics kinematics) {
        this.kinematics = kinematics;
        float closestT = spline.findClosestPointOnSpline(kinematics.payload.pos);
        closest = spline.get(closestT);
        prevClosest = closest.cpy();
        lookaheadPoint = getLookahead(closestT);
        prevLookaheadPoint = lookaheadPoint.cpy();
    }

    public void update() {
        prevClosest = closest;
        prevLookaheadPoint = lookaheadPoint;
        float closestT = spline.findClosestPointOnSpline(kinematics.payload.pos);
        closest = spline.get(closestT);
        lookaheadPoint = getLookahead(closestT);
    }

    public Vector3 getLookahead(float closestT) {
        return spline.get(spline.getTFromLength(spline.getGaussianQuadratureLength(0, closestT) + lookahead));
    }
}
