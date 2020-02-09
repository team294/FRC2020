/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;
import static frc.robot.Constants.*;

/**
 * This class handles all stored robot preferences
 */
public class RobotPreferences {
    private final static Preferences prefs = Preferences.getInstance();

    /**
     * Reads preferences stored in the RoboRIO flash memory and updates their values in Constants.
     * If a preference is not found on the RoboRIO, then this method creates that key on the RoboRio
     * using the default values from Constants.
     * Note:  Any variables in Constants that are read from Preferences must *not* be "final".
     */
	public static void readPreferencesToConstants(){
        // Add a row for each preference to read.  Example:
        // DriveConstants.kA = readDouble("DriveKA", DriveConstants.kA);
        RobotConstants.prototypeBot = readBoolean("prototypeBot", RobotConstants.prototypeBot);
    }

    /**
     * Reads a boolean key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static boolean readBoolean(String keyName, boolean defaultValue) {
		if (!prefs.containsKey(keyName)){
			prefs.putBoolean(keyName, defaultValue);
        } 
        return prefs.getBoolean(keyName, defaultValue);   
    }

    /**
     * Reads a double key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static double readDouble(String keyName, double defaultValue) {
		if (!prefs.containsKey(keyName)){
            prefs.putDouble(keyName, defaultValue);
        } 
        return prefs.getDouble(keyName, defaultValue);   
    }

    /**
     * Reads a string key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static String readString(String keyName, String defaultValue) {
		if (!prefs.containsKey(keyName)){
            prefs.putString(keyName, defaultValue);
        } 
        return prefs.getString(keyName, defaultValue);   
    }
}
