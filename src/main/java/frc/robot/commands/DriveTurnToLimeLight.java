package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class DriveTurnToLimeLight extends CommandBase {
	private final DriveTrain driveTrain;
	private final LimeLight limeLight;
	private boolean isCenter = false;
	private int counter = 0;

	public DriveTurnToLimeLight(DriveTrain driveTrain, LimeLight limeLight) {
		this.driveTrain = driveTrain;
		this.limeLight = limeLight;
		addRequirements(driveTrain);
	}

	public void initialize() {
		isCenter = false;
		counter = 0;
	}

	public void execute() {
		double xOff = limeLight.getXOffset();
		// double yOff = limeLight.getYOffset();
		if (xOff > 0.25) {
			driveTrain.tankDrive(0.45, -0.45);
		} else if (xOff < -0.25) {
			driveTrain.tankDrive(-0.45, 0.45);
		} else {
			isCenter = true;
		}

	}

	public boolean isFinished() {

		if (isCenter == true && counter >= 5) {
			driveTrain.tankDrive(0, 0);
			return true;

		} else if (isCenter == true) {
			counter++;
			return false;
		} else {
			counter = 0;
			return false;
		}

	}

	protected void end() {
		System.out.println("Turn end");
	}

}