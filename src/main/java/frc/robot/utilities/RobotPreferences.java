/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;

/**
 * This class handles all stored robot preferences
 */
public class RobotPreferences {
    private final Preferences prefs;

    public boolean prototypeBot;        // set to true if drive train is reversed from competition robot

    /**
     * Creates a RobotPreferences object and reads the robot preferences.
     */
    public RobotPreferences() {
        prefs = Preferences.getInstance();
        refresh();
    }   

    /**
	 * Re-reads the robot preferences.
	 */
    public void refresh() {
        prototypeBot = prefs.getBoolean("prototypeBot", false);         // This is needed because the competition robot drivetrain is reversed from prototypes
    }

    /* Sets up Preferences if they haven't been set as when changing RoboRios or first start-up.
		The values are set to defaults, so if using the prototype robots set inBCRLab to true
	*/	
	public void doExist(){				 
		if (!prefs.containsKey("prototypeBot")){
			prefs.putBoolean("prototypeBot", false);
        }
    }

}
