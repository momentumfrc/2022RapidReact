package com.momentum4999.robot.subsystems;

import org.usfirst.frc.team4999.lights.AddressableLEDDisplay;
import org.usfirst.frc.team4999.lights.AsyncAnimator;
import org.usfirst.frc.team4999.lights.Color;
import org.usfirst.frc.team4999.lights.ColorTools;
import org.usfirst.frc.team4999.lights.Display;
import org.usfirst.frc.team4999.lights.animations.*;
import org.usfirst.frc.team4999.lights.compositor.AnimationCompositor;

import com.momentum4999.robot.util.Components;

import edu.wpi.first.wpilibj.DriverStation;

public class LEDSubsystem {
	private static final int LED_LENGTH = 240;

	private Display display;
	private AsyncAnimator animator;
	private AnimationCompositor compositor;

	private AnimationCompositor.View baseView;

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
				new Snake(20, rainbowTails),
				5000
			),
			new AnimationSequence.AnimationSequenceMember(
				new Snake(15, ColorTools.getSmearedColors(rainbowcolors, 16)),
				1500
			),
			new AnimationSequence.AnimationSequenceMember(
				new Snake(20, momentumTails),
				5000
			),
			new AnimationSequence.AnimationSequenceMember(
				new Stack(20, 60, rainbowcolors),
				1500
			)
	});

	public LEDSubsystem() {
		try {
			display = new AddressableLEDDisplay(Components.LEDS, LED_LENGTH);
			animator = new AsyncAnimator(display);
			compositor = new AnimationCompositor(animator);
		} catch (Exception e) {
			DriverStation.reportError("Error instantiating LEDSubsystem", e.getStackTrace());
			return;
		}

		baseView = compositor.getOpaqueView(mainAnimation);
	}

}
