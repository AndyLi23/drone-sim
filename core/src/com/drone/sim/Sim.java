package com.drone.sim;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Vector3;

import java.util.ArrayList;
import java.util.Arrays;

public class Sim implements ApplicationListener {
	public PerspectiveCamera cam;
	public ModelBatch modelBatch;
	public Model payload, sphere, ground;
	public Model[] ropes = new Model[4], drones = new Model[4];
	public ModelInstance instance, payloadInstance;
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

		payload = modelBuilder.createBox(kinematics.payloadW, kinematics.payloadH, kinematics.payloadL,
				new Material(ColorAttribute.createDiffuse(Color.GRAY)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		payloadInstance = new ModelInstance(payload);
		payloadInstance.transform.translate(kinematics.payload.pos.x, kinematics.payload.pos.z + kinematics.payloadH / 2, kinematics.payload.pos.y);

		sphere = modelBuilder.createSphere(1f, 1f, 1f, 10, 10, new Material(ColorAttribute.createDiffuse(Color.GREEN)),
				VertexAttributes.Usage.Position | VertexAttributes.Usage.Normal);

		instances.add(new ModelInstance(sphere));
		instances.add(instance);
		instances.add(payloadInstance);

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
		payload.dispose();
	}
}
