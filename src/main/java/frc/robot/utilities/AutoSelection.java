package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.AutoOponentTrenchPickup;
import frc.robot.commands.AutoShootBackup;
import frc.robot.commands.AutoOwnTrenchPickup;
import frc.robot.commands.AutoTrussPickup;
import frc.robot.commands.Wait;
import frc.robot.subsystems.*;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int OPPONENT_TRENCH_PICKUP = 0;
	public static final int SHOOT_BACKUP = 1;
	public static final int TRUSS_PICKUP = 2;
	public static final int OWN_TRENCH_PICKUP = 3;
	

	private Trajectory[] trajectoryCache = new Trajectory[1];
	
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
	public Command getAutoCommand(Integer autoPlan, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LimeLight limeLight, FileLog log, LED led) {
		Command autonomousCommand = null;
		Trajectory trajectory;

		if (autoPlan == OPPONENT_TRENCH_PICKUP && trajectoryCache[OPPONENT_TRENCH_PICKUP] != null) {
			log.writeLogEcho(true, "AutoSelect", "run TrenchFromRight");
			trajectory = trajectoryCache[OPPONENT_TRENCH_PICKUP];
			autonomousCommand = new AutoOponentTrenchPickup(trajectory, driveTrain, limeLight, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == SHOOT_BACKUP) {
			log.writeLogEcho(true, "AutoSelect", "run ShootBackup");
			autonomousCommand = new AutoShootBackup(driveTrain, limeLight, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == TRUSS_PICKUP) {
			log.writeLogEcho(true, "AutoSelect", "run TrussPickup");
			autonomousCommand = new AutoTrussPickup(driveTrain, limeLight, log, shooter, feeder, hopper, intake, led);
		}

		if (autoPlan == OWN_TRENCH_PICKUP) {
			log.writeLogEcho(true, "AutoSelect", "run OwnTrenchPickup");
			autonomousCommand = new AutoOwnTrenchPickup(driveTrain, limeLight, log, shooter, feeder, hopper, intake, led);
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

		if (trajectoryCache[OPPONENT_TRENCH_PICKUP] == null) {
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryOpponentTrenchPickup", "Start");
			trajectoryCache[OPPONENT_TRENCH_PICKUP] = TrajectoryOpponentTrenchToShoot.calcTrajectory(log);
			log.writeLogEcho(true, "AutoSelect", "calcTrajectoryOpponentTrenchPickup", "End");
		}

	}

}
