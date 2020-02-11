package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.AutoTrenchFromCenter;
import frc.robot.commands.AutoTrenchFromRight;
import frc.robot.commands.Wait;
import frc.robot.subsystems.DriveTrain;

/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int TRENCH_FROM_CENTER = 0;
	public static final int TRENCH_FROM_RIGHT = 1;

	private Trajectory[] trajectoryCache = new Trajectory[2];
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(FileLog log) {
		// calc trajectories for later use
		try {
			calcTrajectories(log);
		} catch (Exception e) {
			log.writeLogEcho(true, "AutoSelect", "exception caught in calcTrajectories",e);
		}
		
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard
	 * @param driveTrain  The driveTrain that will be passed to the auto command
	 * @param log The filelog to write the logs to
	 * @param autoPlan The autoplan to run 
	 * @return the command to run
	 */  		
	public Command getAutoCommand(DriveTrain driveTrain, FileLog log, Integer autoPlan) {
		Command autonomousCommand = null;
		Trajectory trajectory;

		if (autoPlan == TRENCH_FROM_RIGHT && trajectoryCache[TRENCH_FROM_RIGHT] != null) {
			log.writeLogEcho(true, "AutoSelect", "run TrenchFromRight");
			trajectory = trajectoryCache[TRENCH_FROM_RIGHT];
			autonomousCommand = new AutoTrenchFromRight(driveTrain, log, trajectory, TrajectoryUtil.getDriveKinematics());
		} 
			
		if (autoPlan == TRENCH_FROM_CENTER && trajectoryCache[TRENCH_FROM_CENTER] != null) {
			log.writeLogEcho(true, "AutoSelect", "run TrenchFromCenter");
			trajectory = trajectoryCache[TRENCH_FROM_CENTER];
			autonomousCommand = new AutoTrenchFromCenter(driveTrain, log, trajectory, TrajectoryUtil.getDriveKinematics());
		}

		if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new Wait(1);
		}

		return autonomousCommand;
	}

	/**
	 * Calculate all the trajectories so they are ready to use before auto period starts
	 */  
	private void calcTrajectories(FileLog log) {

		if (trajectoryCache[TRENCH_FROM_CENTER] == null) {
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryTrenchFromCenter", "Start");
			trajectoryCache[TRENCH_FROM_CENTER] = TrajectoryTrenchFromCenter.calcTrajectory(log);
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryTrenchFromCenter", "End");
		}

		if (trajectoryCache[TRENCH_FROM_RIGHT] == null) {
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryTrenchFromRight", "Start");
			trajectoryCache[TRENCH_FROM_RIGHT] = TrajectoryTrenchFromRight.calcTrajectory(log);
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryTrenchFromRight", "End");
		}

	}

}