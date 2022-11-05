package com.momentum4999.robot.input;

public interface MoInput {
	public double getMoveRequest();
	public double getTurnRequest();

	public boolean getRunShooter();

	public double getElevatorLeft();
	public double getElevatorRight();
}
