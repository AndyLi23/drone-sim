package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Kinematics {
    public Position3 payload;
    public Position3[] drones = new Position3[4];

    public Path3D curPath;

    public static float payloadW, payloadL, payloadH, payloadM, droneXYoffset, droneDiameter;

    public int[] cornerX = new int[]{1, -1, -1, 1};
    public int[] cornerY = new int[]{1, 1, -1, -1};


    public Kinematics(Position3 payload, Path3D curPath,
                      float payloadW, float payloadL, float payloadH, float payloadM,
                      float droneXYoffset, float droneDiameter) {
        this.payload = payload;
        this.curPath = curPath;
        curPath.init(this);

        payload.vel = new Vector3(0.3f, 0, 0.3f);

        Kinematics.payloadW = payloadW; //m
        Kinematics.payloadL = payloadL;
        Kinematics.payloadH = payloadH;

        Kinematics.payloadM = payloadM; //kg

        Kinematics.droneXYoffset = droneXYoffset;
        Kinematics.droneDiameter = droneDiameter;

        for (int i = 0; i < 4; ++i) {
            drones[i] = new Position3();
            drones[i].pos = new Vector3(payloadW / 2f + droneXYoffset, payloadL / 2f + droneXYoffset, 0).scl(
                    cornerX[i], cornerY[i], 0
            ).add(payload.pos);

            drones[i].vel = new Vector3(0, 0, 0.3f);
            drones[i].accel = new Vector3(0, 0, 0f);
        }

    }

    public void update(float dt) {
        payload.update(dt);
        for(int i = 0; i < 4; ++i) {
            drones[i].update(dt);
        }
        curPath.update();
    }

    public Vector3 getDiff(Position3 p) {
        return p.pos.cpy().add(p.prev.cpy().scl(-1));
    }

    public Vector3 getClosestDiff() {
        return curPath.closest.cpy().add(curPath.prevClosest.cpy().scl(-1));
    }

    public Vector3 getLookaheadDiff() {
        return curPath.lookaheadPoint.cpy().add(curPath.prevLookaheadPoint.cpy().scl(-1));
    }

    public static Vector3 toWorld(Vector3 in, float yOffset) {
        return new Vector3(in.x, in.z + yOffset, in.y);
    }

    public static Vector3 toWorldDrone(Vector3 in) {
        return toWorld(in, droneDiameter / 2);
    }

    public static Vector3 toWorldPayload(Vector3 in) {
        return toWorld(in, payloadH / 2);
    }

    public Vector3 getRopeCorner(int i) {
        return new Vector3(payloadW / 2f, payloadL / 2f, payloadH).scl(
                cornerX[i], cornerY[i], 1
        ).add(payload.pos);
    }

    public static Vector3 toWorldNoOffset(Vector3 in) {
        return toWorld(in, 0);
    }
}
