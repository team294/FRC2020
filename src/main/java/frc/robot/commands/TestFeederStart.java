/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class TestFeederStart extends CommandBase {
  double voltage = 6;
  double maxvoltage = 40;
  /**
   * Creates a new FeederHopperIntakeTest.
   */

  private Feeder feeder;

  public TestFeederStart(Feeder feeder) {

    this.feeder = feeder;

    // var timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(feeder);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    feeder.feederSetVoltage(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    feeder.feederGetVoltage();
    if (feeder.feederGetVoltage() <= maxvoltage){
      return true; //finished = true
    }
    return false;
    
    
  }
}
