package com.drone.sim;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.math.Vector3;

import java.util.Arrays;

public class Kinematics {
    public Position3 payload;
    public Position3[] drones = new Position3[4];

    public Path3D curPath;

    public static float payloadW, payloadL, payloadH, payloadM, droneXYoffset, droneDiameter, droneM, ropeLength;

    public int[] cornerX = new int[]{1, -1, -1, 1};
    public int[] cornerY = new int[]{1, 1, -1, -1};

    public Vector3 g = new Vector3(0, 0, -9.807f);


    public Kinematics(Position3 payload, Path3D curPath,
                      float payloadW, float payloadL, float payloadH, float payloadM,
                      float droneM, float droneXYoffset, float droneDiameter, float ropeLength) {
        this.payload = payload;
        this.curPath = curPath;
        curPath.init(this);

//        payload.vel = new Vector3(0.3f, 0, 0.3f);

        Kinematics.payloadW = payloadW; //m
        Kinematics.payloadL = payloadL;
        Kinematics.payloadH = payloadH;

        Kinematics.payloadM = payloadM; //kg

        Kinematics.droneM = droneM;
        Kinematics.droneXYoffset = droneXYoffset;
        Kinematics.droneDiameter = droneDiameter;
        Kinematics.ropeLength = ropeLength;

        for (int i = 0; i < 4; ++i) {
            drones[i] = new Position3();
            drones[i].pos = new Vector3(payloadW / 2f + droneXYoffset, payloadL / 2f + droneXYoffset,
                    payloadH + ropeLength).scl(
                    cornerX[i], cornerY[i], 1
            ).add(payload.pos);
            drones[i].prev = drones[i].pos.cpy();

//            drones[i].vel = new Vector3(0, 0, 0.3f);
//            drones[i].accel = new Vector3(0, 0, 0f);
        }

    }

    public void update(float dt) {

        Vector3 linForce = new Vector3();

        linForce.z = 27f;

        if(Gdx.input.isKeyPressed(Input.Keys.W)) {
            linForce.z += 5f;
        }

        if(Gdx.input.isKeyPressed(Input.Keys.A)) {
            linForce.x -= 5f;
        }

        if(Gdx.input.isKeyPressed(Input.Keys.S)) {
            linForce.z -= 5f;
        }

        if(Gdx.input.isKeyPressed(Input.Keys.D)) {
            linForce.x += 5f;
        }

        physics(new Vector3[]{linForce, linForce, linForce, linForce});

        payload.update(dt);
        for(int i = 0; i < 4; ++i) {
            drones[i].update(dt);
        }
        curPath.update();
    }

    public void physics(Vector3[] droneForces) {
        Vector3 payloadMg = g.cpy().scl(payloadM);
//        if(payload.pos.z <= 1e-6) payloadMg = Vector3.Zero;

        Vector3 droneMg = g.cpy().scl(droneM);

        Vector3 totalForces = new Vector3();
        Vector3 totalTensions = new Vector3();

        for (int i = 0; i < 4; ++i) {
            Vector3 payloadToDrone = drones[i].pos.cpy().add(getRopeCorner(i).scl(-1));
            Vector3 tensionNorm = payloadToDrone.cpy().nor();
            totalTensions.add(tensionNorm);
            double dist = payloadToDrone.len();

//            if(dist < ropeLength) {
//                drones[i].accel = (droneMg.cpy().add(droneForces[i])).scl(1/droneM);
//            } else {
            Vector3 effectiveForce = droneForces[i].cpy().add(droneMg);
            float forceParallelMagnitude = effectiveForce.cpy().dot(tensionNorm);
            Vector3 forceParallel = tensionNorm.cpy().scl(forceParallelMagnitude);

            Vector3 forcePerpendicular = effectiveForce.cpy().sub(forceParallel);

            Vector3 arad = forcePerpendicular.cpy().scl(1 / droneM);


            totalForces.add(forceParallel);

            drones[i].accel = arad;
        }

        payload.accel = totalForces.cpy().add(payloadMg).scl(1/(payloadM)); //subtract drag here

        Vector3 tensions = payload.accel.cpy().scl(payloadM).sub(payloadMg); //add drag here

        float tensionMag = tensions.z / totalTensions.z;

        System.out.println(tensions + " " + totalTensions + " " + totalTensions.cpy().scl(tensionMag));

//        System.out.println(tensionMag);

        for(int i = 0; i < 4; ++i) {
            Vector3 payloadToDrone = drones[i].pos.cpy().add(getRopeCorner(i).scl(-1));
            Vector3 tensionNorm = payloadToDrone.cpy().nor();
            Vector3 tension = tensionNorm.cpy().scl(-tensionMag);

            Vector3 effectiveForce = droneForces[i].cpy().add(droneMg);
            float forceParallelMagnitude = effectiveForce.cpy().dot(tensionNorm);
            Vector3 forceParallel = tensionNorm.cpy().scl(forceParallelMagnitude);

            drones[i].accel.add(forceParallel.cpy().add(tension.cpy()).scl(1/droneM));
//            drones[i].accel.add(payload.accel.cpy());
        }

        System.out.println(drones[0].accel + " " + payload.accel);

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
