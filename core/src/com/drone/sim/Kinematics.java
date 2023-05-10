package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Kinematics {
    public Position3 payload;
    public Position3[] drones = new Position3[4];
    public static float payloadW, payloadL, payloadH, payloadM, droneXYoffset, droneDiameter;

    public int[] cornerX = new int[]{1, -1, -1, 1};
    public int[] cornerY = new int[]{1, 1, -1, -1};

    public Kinematics() {
        payload = new Position3();

//        payload.vel = new Vector3(0.01f, 0, 0);

        payloadW = 1; //m
        payloadL = 1;
        payloadH = 1;

        payloadM = 10; //kg

        droneXYoffset = 0.3f;
        droneDiameter = 0.2f;

        for (int i = 0; i < 4; ++i) {
            drones[i] = new Position3();
            drones[i].pos = new Vector3(payloadW / 2f + droneXYoffset, payloadL / 2f + droneXYoffset, 0).scl(
                    cornerX[i], cornerY[i], 0
            ).add(payload.pos);

            drones[i].vel = new Vector3(0, 0, 0.2f);
            drones[i].accel = new Vector3(0, 0, 0.1f);

            System.out.println(drones[i].pos);
        }


    }

    public void update(float dt) {
        payload.update(dt);
        for(int i = 0; i < 4; ++i) {
            drones[i].update(dt);
        }
    }

    public Vector3 getDiff(Position3 p) {
        return p.pos.cpy().add(p.prev.cpy().scl(-1));
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
