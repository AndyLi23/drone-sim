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
import com.badlogic.gdx.math.Vector3;

import java.util.ArrayList;

public class Sim implements ApplicationListener {
	public PerspectiveCamera cam;
	public ModelBatch modelBatch;
	public Model payloadModel, sphere, groundModel;
	public ModelInstance[] ropes = new ModelInstance[4], drones = new ModelInstance[4], forces = new ModelInstance[4];
	public ModelInstance ground, payload, closest, lookahead;
	public ArrayList<ModelInstance> instances = new ArrayList<>();
	public Environment environment;
	public ModelBuilder modelBuilder;

	public CameraInputController camController;

	public int ind = 0;

	public static Kinematics kinematics;

	public static boolean path = true;
	
	@Override
	public void create () {
		Gdx.graphics.setWindowedMode(1200, 800);

		initializeKinematics();

		modelBatch = new ModelBatch();

		cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		cam.position.set(5f, 1f, -5f);
		cam.lookAt(0,0,0);
		cam.near = 1f;
		cam.far = 300f;
		cam.update();

		modelBuilder = new ModelBuilder();

		groundModel = modelBuilder.createBox(100, 0.01f, 100, new Material(ColorAttribute.createDiffuse(0.8f, 0.8f, 0.8f, 1f)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		ground = new ModelInstance(groundModel);
		ground.transform.translate(0, -0.01f, 0);
		instances.add(ground);

		for(int x = -50; x <= 50; ++x) {
			modelBuilder.begin();
			MeshPartBuilder builder = modelBuilder.part("line", 1, 3, new Material());
			builder.setColor(Color.DARK_GRAY);
			builder.line(x, 0.0f, -50, x, 0.0f, 50);
			Model lineModel = modelBuilder.end();
			instances.add(new ModelInstance(lineModel));
		}

		for(int z = -50; z <= 50; ++z) {
			modelBuilder.begin();
			MeshPartBuilder builder = modelBuilder.part("line", 1, 3, new Material());
			builder.setColor(Color.DARK_GRAY);
			builder.line(-50, 0.0f, z, 50, 0.0f, z);
			Model lineModel = modelBuilder.end();
			instances.add(new ModelInstance(lineModel));
		}

		if (path)
			renderSpline();

		payloadModel = modelBuilder.createBox(Kinematics.payloadW, Kinematics.payloadH, Kinematics.payloadL,
				new Material(ColorAttribute.createDiffuse(Color.GRAY)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		payload = new ModelInstance(payloadModel);
		payload.transform.translate(Kinematics.toWorldPayload(kinematics.payload.pos));
		instances.add(payload);

		sphere = modelBuilder.createSphere(0.2f, 0.2f, 0.2f,
				10, 10, new Material(ColorAttribute.createDiffuse(Color.YELLOW)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		closest = new ModelInstance(sphere);
		closest.transform.translate(Kinematics.toWorldNoOffset(kinematics.curPath.closest));
		if (path) instances.add(closest);

		sphere = modelBuilder.createSphere(0.2f, 0.2f, 0.2f,
				10, 10, new Material(ColorAttribute.createDiffuse(Color.ORANGE)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		lookahead = new ModelInstance(sphere);
		lookahead.transform.translate(Kinematics.toWorldNoOffset(kinematics.curPath.lookaheadPoint));
		if (path) instances.add(lookahead);

		sphere = modelBuilder.createSphere(Kinematics.droneDiameter, Kinematics.droneDiameter, Kinematics.droneDiameter,
				10, 10, new Material(ColorAttribute.createDiffuse(Color.BLUE)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		for (int i = 0; i < 4; ++i) {
			drones[i] = new ModelInstance(sphere);
			drones[i].transform.translate(Kinematics.toWorldDrone(kinematics.drones[i].pos));
			instances.add(drones[i]);
		}

		for (int i = 0; i < 4; ++i) {
			ropes[i] = createRope(i);
			instances.add(ropes[i]);
		}

		for (int i = 0; i < 4; ++i) {
			forces[i] = createForce(i, 0);
			instances.add(forces[i]);
		}

//		instances.add(new ModelInstance(sphere));

		environment = new Environment();
		environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
		environment.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -0.2f, -0.8f, -1f));

		camController = new CameraInputController(cam);
//		Gdx.input.setInputProcessor(camController);

//		instances.add(cylinder(kinematics.curPath.spline.pos0, kinematics.curPath.spline.pos1, 0.01f, Color.RED));
	}

	public void initializeKinematics() {
		kinematics = new Kinematics(new Position3(),
				new Path3D(
						new CubicHermite3D(
								new Vector3(0, 0, 0),
								new Vector3(30, 30, 10),
								new Vector3(0, 0, 10),
								new Vector3(10, 10, 0)
						), 1f, 0.5f, 0.25f, 0.25f
				),
				0.5f, 0.5f, 0.5f, 5f,
				1f,
				0.2f, 0.5f);
	}

	public void updateKinematics() {
		kinematics.update(ind);
		kinematics.updatePath(ind, this);

		payload.transform.translate(Kinematics.toWorldNoOffset(kinematics.getDiff(kinematics.payload)));
		cam.translate(Kinematics.toWorldNoOffset(kinematics.getDiff(kinematics.payload)));
		cam.lookAt(Kinematics.toWorldNoOffset(kinematics.payload.pos));
		cam.update();
		for (int i = 0; i < 4; ++i) {
			drones[i].transform.translate(Kinematics.toWorldNoOffset(kinematics.getDiff(kinematics.drones[i])));

			instances.remove(ropes[i]);
			ropes[i] = createRope(i);
			instances.add(ropes[i]);

			instances.remove(forces[i]);
			forces[i] = createForce(i, ind);
			instances.add(forces[i]);
		}
		closest.transform.translate(Kinematics.toWorldNoOffset(kinematics.getClosestDiff()));
		lookahead.transform.translate(Kinematics.toWorldNoOffset(kinematics.getLookaheadDiff()));

		ind+= (path ? 2 : 5);
	}

	@Override
	public void resize(int width, int height) {

	}

	@Override
	public void render () {
		//WORLD: (x, y, z)
		//LIBGDX: (x, z, y)

		camController.update();

		Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT |
				(Gdx.graphics.getBufferFormat().coverageSampling?GL20.GL_COVERAGE_BUFFER_BIT_NV:0));
//		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

		modelBatch.begin(cam);
		modelBatch.render(instances, environment);
		modelBatch.end();

		updateKinematics();
	}

	public ModelInstance createRope(int i) {
		return cylinder(
				kinematics.drones[i].pos,
				kinematics.getRopeCorner(i),
				0.015f, Color.RED, VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal
		);
	}

	public ModelInstance createForce(int i, int ind) {
		return cylinder(
				kinematics.drones[i].pos,
				kinematics.getForceEnd(i, ind),
				0.025f, new Color(1f, 0.7f, 0, 1), VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal
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

		if (!(from.x == to.x && from.z == to.z)) {
			Vector3 n = to.cpy().add(from.cpy().scl(-1));
			Vector3 cs = n.cpy().crs(Vector3.Y).nor();
			Vector3 ac = n.cpy().crs(cs).nor();

			ins.transform.rotateTowardDirection(ac, Vector3.Y);
		}
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
