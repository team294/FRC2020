/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

//NOTE: NEED WAY TO GET PISTON POSITION FOR LOCK AND HOOD

public class TestShooterFinish extends CommandBase {
  /**
   * Creates a new Shootertest.
   */

  private Shooter shooter;

  private int rpm;
  public TestShooterFinish(Shooter shooter) {
    this.shooter = shooter;
  
    // Use addRequirements() here to declare subsystem dependencies.
   
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = (int)shooter.getMeasuredRPM();
    
      if(rpm < 1700)
      {
        System.out.println("Shooter is too slow. RPM is: " + rpm);
      } 
      if(rpm > 2300) 
      {
      System.out.println("Shooter is too fast. RPM is: " + rpm);
      }
     // if(shooter.getLockPiston)
     // {
     //   shooter.SetLockPiston(false);
     //   System.out.Println("LockPiston Working")
    //  }
    ///  if(shooter.getHoodPistontrue)
    //  {
      
    //  shooter.SetHoodPiston(true);
   //   System.out.println("HoodPiston Working");
   //   }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    shooter.setShooterPID(0);
    shooter.setHoodPiston(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
