package com.momentum4999.robot.subsystems;

import org.usfirst.frc.team4999.lights.Animator;
import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.ColorTools;
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

	private static Color[] rainbowcolors = {
		new Color(72, 21, 170),
		new Color(55, 131, 255),
		new Color(77, 233, 76),
		new Color(255, 238, 0),
		new Color(255, 140, 0),
		new Color(246, 0, 0)
	};


	Color[] rainbowTails = ColorTools.getColorTails(rainbowcolors, Color.BLACK, 12, 20);
	Color[] momentumTails = ColorTools.getColorTails(
		new Color[] {Color.MOMENTUM_BLUE, Color.MOMENTUM_PURPLE},
		Color.BLACK, 24, 32
	);

	Animation mainAnimation = new AnimationSequence(
		new AnimationSequence.AnimationSequenceMember[] {
			new AnimationSequence.AnimationSequenceMember(
				new Snake(rainbowTails, 7),
				5000
			),
			new AnimationSequence.AnimationSequenceMember(
				new Snake(ColorTools.getSmearedColors(rainbowcolors, 16), 5),
				1500
			),
			new AnimationSequence.AnimationSequenceMember(
				new Snake(momentumTails, 7),
				5000
			),
			new AnimationSequence.AnimationSequenceMember(
				new Stack(rainbowcolors, 20, 20),
				1500
			)
	});

	public LEDSubsystem() {

		try {
			display = NeoPixels.getInstance(Port.kMXP);
			animator = new Animator(display);
			compositor = new AnimationCompositor(animator);

			setBaseAnimation(mainAnimation);
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