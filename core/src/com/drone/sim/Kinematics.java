package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

import java.util.Arrays;

public class Kinematics {
    public Position3 payload;
    public Position3[] drones = new Position3[4];
    public static float payloadW, payloadL, payloadH, payloadM, droneXYoffset, droneDiameter;

    public int[] droneCornerX = new int[]{1, -1, -1, 1};
    public int[] droneCornerY = new int[]{1, 1, -1, -1};

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
            drones[i].pos = new Vector3(payloadW / 2f + droneXYoffset, payloadH / 2f + droneXYoffset, 0).scl(
                    droneCornerX[i], droneCornerY[i], 0
            );

            System.out.println(drones[i].pos);
        }


    }

    public static Vector3 toWorldDrone(Vector3 in) {
        return new Vector3(in.x, in.z + droneDiameter / 2, in.y);
    }

    public static Vector3 toWorldPayload(Vector3 in) {

    }

    public static Vector3 toWorldRope(Vector3 in) {
        
    }
}
