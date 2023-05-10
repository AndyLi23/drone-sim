package com.drone.sim;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.glutils.HdpiMode;
import com.drone.sim.Sim;

// Please note that on macOS your application needs to be started with the -XstartOnFirstThread JVM argument
public class DesktopLauncher {
	public static void main (String[] arg) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
		config.setHdpiMode(HdpiMode.Pixels);
		config.setForegroundFPS(60);
		config.setTitle("drone-sim");
		new Lwjgl3Application(new Sim(), config);
	}
}
