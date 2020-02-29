/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShootSequenceSetup extends SequentialCommandGroup {
  
  /**
   * Set the shooter hood position (open or close, lock or unlock).
   * Then set the shooter RPM either with the distance from the target or the default short shot RPM.
   * @param closeHood true = close the hood, false = open the hood
   * @param shooter shooter subsystem
   * @param limeLight limelight camera (subsystem)
   * @param led led strip (subsystem)
   */
  public ShootSequenceSetup(boolean closeHood, Shooter shooter, LimeLight limeLight, LED led, FileLog log) {
    addCommands(
      // If the current distance away from the target is greater than the max distance for 
      // unlocking the hood, close and lock the hood. Otherwise, close the hood and leave it unlocked.
      new ConditionalCommand(
        new ShooterHoodPistonSequence(closeHood, true, shooter, log),
        new ShooterHoodPistonSequence(closeHood, false, shooter, log),
        () -> closeHood && limeLight.getDistanceNew() > LimeLightConstants.unlockedHoodMaxDistance
      ),
      // If closing the hood, set shooter RPM based on distance.
      // Otherwise, set shooter RPM to the default value for the short shot.
      new ConditionalCommand(
        new ShooterSetPID(true, false, shooter, limeLight, led, log),
        new ShooterSetPID(ShooterConstants.shooterDefaultShortRPM, shooter, led, log),
        () -> closeHood
      )
    );
  }
}
