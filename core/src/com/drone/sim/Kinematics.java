package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class Kinematics {
    public Position3 payload;
    public float payloadW, payloadL, payloadH, payloadM;

    public Kinematics() {
        payload = new Position3();

        payloadW = 1; //m
        payloadL = 1;
        payloadH = 1;

        payloadM = 10; //kg
    }
}
