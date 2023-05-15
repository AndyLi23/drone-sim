package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Position3 {
    public Vector3 pos, vel, accel;
    public Vector3 prev;

    public Position3() {
        this(new Vector3(), new Vector3(), new Vector3());
    }

    public Position3(Vector3 pos, Vector3 vel, Vector3 accel) {
        this.pos = pos;
        this.vel = vel;
        this.accel = accel;

        this.prev = pos.cpy();
    }

    public void update(Vector3 new_) {
        this.prev = pos.cpy();
        this.pos = new_;
    }

    public void update(float dt) {
        this.prev = pos.cpy();

        pos.add(vel.cpy().scl(dt).add(accel.cpy().scl(0.5f * dt * dt)));
        vel.add(accel.cpy().scl(dt));
    }
}
