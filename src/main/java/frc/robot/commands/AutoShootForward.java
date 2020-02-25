/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

public class AutoShootForward extends SequentialCommandGroup {

  // multiplier to adjust max values down to a safe percent to use when driving
  private static final double MAX_ADJUSTMENT = 0.6; 

  /**
   * Creates a command group that waits a specified time, shoots and then moves forward out of the way. 
   * Can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target
   * 
   * @param waitTime       the time in seconds to wait before starting
   * @param driveTrain     drivetrain subsystem to use
   * @param limeLight      limelight subsystem to use
   * @param log            log subsystem to use
   * @param shooter        shooter subsystem to use
   * @param feeder         feeder subsystem to use
   * @param hopper         hopper subsystem to use
   * @param intake         intake subsystem to use
   * @param led            intake subsystem to use
   */
  public AutoShootForward(double waitTime, DriveTrain driveTrain, LimeLight limeLight, FileLog log, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, LED led) {

    // can start anywhere on auto line between left most pole from driver perspective and close to right edge of the field, needs to be semi lined up with target

    addCommands(

      new Wait(waitTime),

      // turn towards target w/ vision
      deadline(
        new DriveTurnGyro(TargetType.kVision, 
          0, 
          DriveConstants.kMaxAngularVelocity * MAX_ADJUSTMENT, 
          DriveConstants.kMaxAngularAcceleration * MAX_ADJUSTMENT, 
          0.8, 
          driveTrain, 
          limeLight, 
          log).withTimeout(DriveConstants.maxSecondsForTurnGyro), 
        new ShooterSetPID(2800, shooter, led), // start shooter
        new IntakePistonSetPosition(true, intake) // deploy intake piston
      ),

      // start shooter and wait for 3 power cells to be shot
      deadline(
        new WaitForPowerCells(3, shooter).withTimeout(ShooterConstants.maxSecondsToShoot3balls), 
        new ShootSequence(2800, shooter, feeder, hopper, intake, led) 
      ),
      
      // stop all motors
      new ShootSequenceStop(shooter, feeder, hopper, intake, led).withTimeout(0.1), 
      
      // go forward 2 meters to get off auto line
      new DriveStraight(2,
        TargetType.kRelative, 
        0,
        DriveConstants.kMaxSpeedMetersPerSecond * MAX_ADJUSTMENT, 
        DriveConstants.kMaxAccelerationMetersPerSecondSquared * MAX_ADJUSTMENT, 
        true, 
        driveTrain, 
        limeLight,
        log) 

    );
  }
}
