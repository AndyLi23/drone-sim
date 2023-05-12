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

    public void update(float dt) {
        Vector3 temp = pos.cpy();
        pos.add(pos.cpy().add(prev.cpy().scl(-1)).add(accel.cpy().scl(dt * dt)));
        prev = temp;
//        vel.add(accel.cpy().scl(dt));
//        pos.add(vel.cpy().scl(dt));
    }

    public Vector3 vel() {
        return pos.cpy().add(prev.cpy().scl(-1));
    }
}