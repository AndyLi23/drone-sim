package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class CubicHermite3D {
    public Vector3 pos0, pos1, vel0, vel1;

    public CubicHermite3D(Vector3 pos0, Vector3 pos1, Vector3 vel0, Vector3 vel1) {
        this.pos0 = pos0;
        this.pos1 = pos1;
        this.vel0 = vel0;
        this.vel1 = vel1;
    }

    public Vector3 get(float t) {
        return pos0.cpy().scl(basis(t, 0))
                .add(pos1.cpy().scl(basis(t, 3)))
                .add(vel0.cpy().scl(basis(t, 1)))
                .add(vel1.cpy().scl(basis(t, 2)));
    }

    public float basis(float t, int i) {
        switch(i) {
            case 0:
                return 1 - 3 * t * t + 2 * t * t * t;
            case 1:
                return t - 2 * t * t + t * t * t;
            case 2:
                return -t * t + t * t * t;
            case 3:
                return 3 * t * t - 2 * t * t * t;
            default:
                return 0;
        }
    }
}
