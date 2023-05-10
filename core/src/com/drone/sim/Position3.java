package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Position3 {
    public Vector3 pos, vel, accel;

    public Position3() {
        this(new Vector3(), new Vector3(), new Vector3());
    }

    public Position3(Vector3 pos, Vector3 vel, Vector3 accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;
    }
}
