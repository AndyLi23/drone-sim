package com.drone.sim;

import com.badlogic.gdx.Graphics;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.glutils.HdpiMode;
import com.drone.sim.Sim;

import java.awt.*;

// Please note that on macOS your application needs to be started with the -XstartOnFirstThread JVM argument
public class DesktopLauncher {
	public static void main (String[] arg) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
		config.setHdpiMode(HdpiMode.Pixels);
		config.setForegroundFPS(100);
		config.setTitle("drone-sim");
		config.setBackBufferConfig(8,8,8,8,16,0,3);
		new Lwjgl3Application(new Sim(), config);
	}
}
