package com.momentum4999.robot.subsystems;

import org.usfirst.frc.team4999.lights.Animator;
import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.Display;
import org.usfirst.frc.team4999.lights.NeoPixels;
import org.usfirst.frc.team4999.lights.animations.*;
import org.usfirst.frc.team4999.lights.compositor.AnimationCompositor;
import org.usfirst.frc.team4999.lights.compositor.FullScreenView;

import edu.wpi.first.wpilibj.I2C.Port;

public class LEDSubsystem {
	private Display display;
	private Animator animator;
	private AnimationCompositor compositor;

	private static final String BASE_ANIMATION_KEY = "BASE_ANIMATION";
	private static final int BASE_ANIMATION_PRIORITY = 0;

	private static Color[] rainbowcolors = { new Color(139, 0, 255), Color.BLUE, Color.GREEN, Color.YELLOW,
			new Color(255, 127, 0), Color.RED };

	public final Animation rainbow = new AnimationSequence(new Animation[] { Snake.rainbowSnake(70),
			Fade.rainbowFade(100, 20), new Bounce(Color.WHITE, rainbowcolors, 40, 50), new Stack(rainbowcolors, 50, 40),
			new BounceStack(rainbowcolors, 20, 40) }, new int[] { 5000, 5000, 10000, 10000, 10000 });

	public LEDSubsystem() {

		try {
			display = NeoPixels.getInstance(Port.kMXP);
			animator = new Animator(display);
			compositor = new AnimationCompositor(animator);

			setBaseAnimation(rainbow);
		} catch (Exception e) {
			e.printStackTrace();
			compositor = null;
			if (animator != null) {
				animator.stopAnimation();
			}
			animator = null;
			display = null;
		}

	}

	public void setBaseAnimation(Animation animation) {
		if (compositor != null) {
			compositor.hideView(BASE_ANIMATION_KEY);
			compositor.showView(BASE_ANIMATION_KEY, new FullScreenView(animation), BASE_ANIMATION_PRIORITY);
		}
	}

}