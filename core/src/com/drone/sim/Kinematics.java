package com.drone.sim;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.math.Vector3;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import java.util.Arrays;

public class Kinematics {
    public Position3 payload;
    public Position3[] drones = new Position3[4];

    public Path3D curPath;

    public static float payloadW, payloadL, payloadH, payloadM, ropeLength, droneDiameter, droneM;

    public int[] cornerX = new int[]{1, -1, -1, 1};
    public int[] cornerY = new int[]{1, 1, -1, -1};

    public ArrayList<Vector3> payloadPos = new ArrayList<>();
    public ArrayList<ArrayList<Vector3>> dronePos = new ArrayList<>();
    public ArrayList<Vector3> forces = new ArrayList<>();

    public Kinematics(Position3 payload, Path3D curPath,
                      float payloadW, float payloadL, float payloadH, float payloadM,
                      float ropeLength, float droneDiameter, float droneM) {
        this.payload = payload;
        this.curPath = curPath;
        curPath.init(this);

//        payload.vel = new Vector3(0.3f, 0, 0.3f);

        Kinematics.payloadW = payloadW; //m
        Kinematics.payloadL = payloadL;
        Kinematics.payloadH = payloadH;

        Kinematics.payloadM = payloadM; //kg

        Kinematics.ropeLength = ropeLength;
        Kinematics.droneDiameter = droneDiameter;
        Kinematics.droneM = droneM;

        for (int i = 0; i < 4; ++i) {
            drones[i] = new Position3();
            drones[i].pos = new Vector3(payloadW / 2f, payloadL / 2f, payloadH + ropeLength).scl(
                    cornerX[i], cornerY[i], 1
            ).add(payload.pos);
            drones[i].prev = drones[i].pos.cpy();

//            drones[i].vel = new Vector3(0, 0, 0.3f);
//            drones[i].accel = new Vector3(0, 0, 0f);
        }

        try {
            loadData();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }

    }

    public void loadData() throws IOException, ParseException {
        JSONObject obj = (JSONObject) new JSONParser().parse(new FileReader("assets/" + (Sim.path ? "data-pf" : "data-rand") + ".json"));
        JSONArray pl = (JSONArray) obj.get("payload");
        for (Object a : pl.toArray()) {
            JSONArray temp = (JSONArray) a;
            payloadPos.add(new Vector3(Float.parseFloat(String.valueOf(temp.get(0))),
                    Float.parseFloat(String.valueOf(temp.get(1))),
                    Float.parseFloat(String.valueOf(temp.get(2))) - 0.5f));

            forces.add(new Vector3(Float.parseFloat(String.valueOf(temp.get(3))),
                    Float.parseFloat(String.valueOf(temp.get(4))),
                    Float.parseFloat(String.valueOf(temp.get(5)))));
        }

        for (int i = 1; i < 5; ++i) {
            pl = (JSONArray) obj.get("drone" + i);
            ArrayList<Vector3> ta = new ArrayList<>();

            for (Object a : pl.toArray()) {
                JSONArray temp = (JSONArray) a;
                ta.add(new Vector3(Float.parseFloat(String.valueOf(temp.get(0))),
                        Float.parseFloat(String.valueOf(temp.get(1))),
                        Float.parseFloat(String.valueOf(temp.get(2)))));
            }

            dronePos.add(ta);
        }
    }

    public void update(int ind) {
        if (ind < payloadPos.size()) {
            payload.update(payloadPos.get(ind));
            for (int i = 0; i < 4; ++i) drones[i].update(dronePos.get(i).get(ind));
            curPath.update();
        } else {
            payload.prev = payload.pos.cpy();
            for (int i = 0; i < 4; ++i) drones[i].prev = drones[i].pos.cpy();
            curPath.prevLookaheadPoint = curPath.lookaheadPoint.cpy();
            curPath.prevClosest = curPath.closest.cpy();
        }
    }

    public void updatePath(int ind, Sim sim) {
        if (ind < payloadPos.size()) {
            sim.instances.add(sim.cylinder(payload.pos, payload.prev, 0.02f, new Color(0f, 0.8f, 1f, 1f),
                    VertexAttributes.Usage.Position));
        }
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

    public Vector3 getForceEnd(int i, int ind) {
        if (ind < forces.size()) {
            return drones[i].pos.cpy().add(forces.get(ind).cpy().scl(0.03f));
        } else {
            return drones[i].pos.cpy().add(forces.get(forces.size() - 1).cpy().scl(0.03f));
        }
    }

    public static Vector3 toWorldNoOffset(Vector3 in) {
        return toWorld(in, 0);
    }
}
