package com.drone.sim;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;

import java.util.ArrayList;

public class Sim implements ApplicationListener {
	public PerspectiveCamera cam;
	public ModelBatch modelBatch;
	public Model payloadModel, sphere, ground;
	public ModelInstance[] ropes = new ModelInstance[4], drones = new ModelInstance[4];
	public ModelInstance instance, payload;
	public ArrayList<ModelInstance> instances = new ArrayList<>();
	public Environment environment;

	public CameraInputController camController;



	public static Kinematics kinematics = new Kinematics();
	
	@Override
	public void create () {
		modelBatch = new ModelBatch();

		cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		cam.position.set(4f, 4f, 4f);
		cam.lookAt(0,0,0);
		cam.near = 1f;
		cam.far = 300f;
		cam.update();

		ModelBuilder modelBuilder = new ModelBuilder();

		ground = modelBuilder.createBox(1000, 0.01f, 1000, new Material(ColorAttribute.createDiffuse(0.2f, 0.2f, 0.2f, 1f)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		instance = new ModelInstance(ground);
		instance.transform.translate(0, -0.01f, 0);

		payloadModel = modelBuilder.createBox(kinematics.payloadW, kinematics.payloadH, kinematics.payloadL,
				new Material(ColorAttribute.createDiffuse(Color.GRAY)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		payload = new ModelInstance(payloadModel);
		payload.transform.translate(kinematics.payload.pos.x, kinematics.payload.pos.z + kinematics.payloadH / 2, kinematics.payload.pos.y);

		sphere = modelBuilder.createSphere(kinematics.droneDiameter, kinematics.droneDiameter, kinematics.droneDiameter,
				10, 10, new Material(ColorAttribute.createDiffuse(Color.BLUE)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		for (int i = 0; i < 4; ++i) {
			drones[i] = new ModelInstance(sphere);
			drones[i].transform.translate(Kinematics.toWorldDrone(kinematics.drones[i].pos));
			instances.add(drones[i]);
		}

//		instances.add(new ModelInstance(sphere));
		instances.add(instance);
		instances.add(payload);

		environment = new Environment();
		environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
		environment.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -0.2f, -0.8f, -1f));

		camController = new CameraInputController(cam);
		Gdx.input.setInputProcessor(camController);
	}

	@Override
	public void resize(int width, int height) {

	}

	@Override
	public void render () {
		//WORLD: (x, y, z)
		//LIBGDX: (x, z, y)

		camController.update();

//		payloadInstance.transform.translate(kinematics.payload.vel);

		Gdx.gl.glViewport(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

//		Vector3 pos = new Vector3();
//		payloadInstance.transform.getTranslation(pos);
//		System.out.println(pos);

		modelBatch.begin(cam);
		modelBatch.render(instances, environment);
		modelBatch.end();
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
