package com.drone.sim;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Quaternion;
import com.badlogic.gdx.math.Vector3;

import java.util.ArrayList;

public class Sim implements ApplicationListener {
	public PerspectiveCamera cam;
	public ModelBatch modelBatch;
	public Model payloadModel, sphere, groundModel;
	public ModelInstance[] ropes = new ModelInstance[4], drones = new ModelInstance[4];
	public ModelInstance ground, payload;
	public ArrayList<ModelInstance> instances = new ArrayList<>();
	public Environment environment;
	public ModelBuilder modelBuilder;

	public CameraInputController camController;



	public static Kinematics kinematics;
	
	@Override
	public void create () {
		initializeKinematics();


		modelBatch = new ModelBatch();

		cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		cam.position.set(4f, 4f, 4f);
		cam.lookAt(0,0,0);
		cam.near = 1f;
		cam.far = 300f;
		cam.update();

		modelBuilder = new ModelBuilder();

		groundModel = modelBuilder.createBox(1000, 0.01f, 1000, new Material(ColorAttribute.createDiffuse(0.2f, 0.2f, 0.2f, 1f)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		ground = new ModelInstance(groundModel);
		ground.transform.translate(0, -0.01f, 0);
		instances.add(ground);

		renderSpline();

		payloadModel = modelBuilder.createBox(kinematics.payloadW, kinematics.payloadH, kinematics.payloadL,
				new Material(ColorAttribute.createDiffuse(Color.GRAY)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		payload = new ModelInstance(payloadModel);
		payload.transform.translate(Kinematics.toWorldPayload(kinematics.payload.pos));
		instances.add(payload);

		sphere = modelBuilder.createSphere(kinematics.droneDiameter, kinematics.droneDiameter, kinematics.droneDiameter,
				10, 10, new Material(ColorAttribute.createDiffuse(Color.BLUE)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		for (int i = 0; i < 4; ++i) {
			drones[i] = new ModelInstance(sphere);
			drones[i].transform.translate(Kinematics.toWorldDrone(kinematics.drones[i].pos));
			instances.add(drones[i]);
		}

		for(int i = 0; i < 4; ++i) {
			ropes[i] = createRope(i);
			instances.add(ropes[i]);
		}

//		instances.add(new ModelInstance(sphere));

		environment = new Environment();
		environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
		environment.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -0.2f, -0.8f, -1f));

		camController = new CameraInputController(cam);
		Gdx.input.setInputProcessor(camController);

//		instances.add(cylinder(kinematics.curPath.spline.pos0, kinematics.curPath.spline.pos1, 0.01f, Color.RED));
	}

	public void initializeKinematics() {
		kinematics = new Kinematics(new Position3(),
				new Path3D(
						new CubicHermite3D(
								new Vector3(0, 0, 0),
								new Vector3(5, 0, 5),
								new Vector3(0, 0, 5),
								new Vector3(0, 5, 0)
						), 0.5f, 0.25f, 0.25f
				),
				1f, 1f, 1f,
				10f, 0.3f, 0.2f);
	}

	public void updateKinematics() {
		float dt = 1 / 60f;

		kinematics.update(dt);

		payload.transform.translate(Kinematics.toWorldNoOffset(kinematics.getDiff(kinematics.payload)));
		for (int i = 0; i < 4; ++i) {
			drones[i].transform.translate(Kinematics.toWorldNoOffset(kinematics.getDiff(kinematics.drones[i])));

			instances.remove(ropes[i]);
			ropes[i] = createRope(i);
			instances.add(ropes[i]);
		}
	}

	@Override
	public void resize(int width, int height) {

	}

	@Override
	public void render () {
		//WORLD: (x, y, z)
		//LIBGDX: (x, z, y)

		camController.update();

		updateKinematics();

		Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

		modelBatch.begin(cam);
		modelBatch.render(instances, environment);
		modelBatch.end();
	}

	public ModelInstance createRope(int i) {
		return cylinder(
				kinematics.drones[i].pos,
				kinematics.getRopeCorner(i),
				0.04f, Color.RED, VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal
		);
	}

	public void renderSpline() {
		CubicHermite3D spline = kinematics.curPath.spline;
		float step = 0.025f;
		for(float t = step; t <= 1 + 1e-6; t += step) {
			instances.add(cylinder(spline.get(t - step), spline.get(t), 0.02f, Color.GREEN,
					VertexAttributes.Usage.Position));
		}
	}

	public ModelInstance cylinder(Vector3 from, Vector3 to, float radius, Color color, long attributes) {
		from = Kinematics.toWorldNoOffset(from);
		to = Kinematics.toWorldNoOffset(to);
		float dist = from.dst(to);
		Model cyl = modelBuilder.createCylinder(radius * 2, dist, radius * 2, 8, new Material(
				ColorAttribute.createDiffuse(color)), attributes);
		ModelInstance ins = new ModelInstance(cyl);
		ins.transform.translate(to.cpy().add(from).scl(0.5f));

		Vector3 n = to.cpy().add(from.cpy().scl(-1));
		Vector3 cs = n.cpy().crs(Vector3.Y).nor();
		Vector3 ac = n.cpy().crs(cs).nor();

		ins.transform.rotateTowardDirection(ac, Vector3.Y);


//		ins.transform.rotate(ins.transform.rot, 90f);
//		Quaternion q = new Quaternion();
//		ins.transform.getRotation(q);
		return ins;
	}

	@Override
	public void pause() {

	}

	@Override
	public void resume() {

	}

	@Override
	public void dispose () {
		modelBatch.dispose();
		payloadModel.dispose();
	}
}
