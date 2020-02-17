/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class used to write information to file for logging.
 *
 */
public class StickyFaults {
	private FileLog log;
	public String problemSubsystem;
	public boolean problemExists;

	public StickyFaults(FileLog log) {
		this.problemSubsystem = "";
		this.problemExists = false;
		RobotPreferences.editStickyFaults(problemSubsystem, problemExists);
		this.log = log;
	}

	/**
	 * Records in robotPreferences, fileLog, and Shuffleboard that a problem was found in a subsystem
	 * (only records if the subsystem wasn't already flagged)
	 * @param subsystem String name of subsystem in which a problem exists
	 */
	public void recordStickyFaults(String subsystem) {
		if (problemSubsystem.indexOf(subsystem) == -1) {
			if (problemSubsystem.length() != 0) {
				problemSubsystem = problemSubsystem + ", ";
			}
			problemSubsystem = problemSubsystem + subsystem;
			log.writeLogEcho(true, subsystem, "Sticky Faults Logged", "");
		}
		if (!problemExists) {
			problemExists = true;
		}
		RobotPreferences.editStickyFaults(problemSubsystem, problemExists);
		showStickyFaults();
	}
	
	/**
	 * Clears any sticky faults in the RobotPreferences and Shuffleboard
	 */
	public void clearStickyFaults() {
		problemSubsystem = "";
		problemExists = false;
		RobotPreferences.editStickyFaults(problemSubsystem, problemExists);
		showStickyFaults();
		log.writeLog(true, "StickyFaults", "Sticky Faults Cleared", "");
	}

	/**
	 * Show any sticky faults on Shuffleboard
	 */
	public void showStickyFaults() {
		SmartDashboard.putString("problemSubsystem", problemSubsystem);
		SmartDashboard.putBoolean("problemExists", problemExists);
	}

}