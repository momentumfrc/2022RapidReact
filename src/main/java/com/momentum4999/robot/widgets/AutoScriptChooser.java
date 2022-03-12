package com.momentum4999.robot.widgets;

import com.momentum4999.robot.RobotContainer;
import com.momentum4999.robot.util.MoCode;
import com.momentum4999.robot.util.MoShuffleboard;
import com.momentum4999.robot.util.MoCode.MoCodeRuntime;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class AutoScriptChooser extends SendableChooser<String> {
	public AutoScriptChooser(RobotContainer robot) {
		super();

		String defaultScript = MoCode.INSTANCE.getDefaultScriptName();
		this.setDefaultOption(defaultScript, defaultScript);

		for (String scriptName : MoCode.INSTANCE.getLoadedScripts()) {
			this.addOption(scriptName, scriptName);
		}

		MoShuffleboard.matchTab().add("Autonomous Script", this).withWidget(BuiltInWidgets.kSplitButtonChooser).withPosition(0, 1).withSize(4, 1);
	}

	public MoCodeRuntime getSelectedRuntime() {
		return MoCode.INSTANCE.getRunnableScript(this.getSelected());
	}
}
