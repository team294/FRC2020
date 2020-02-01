/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.*;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */

  private FileLog log;

  private final WPI_TalonFX climbMotor1;
  private final WPI_TalonFX climbMotor2;
  private final Solenoid climbPiston1;
  private final Solenoid climbPiston2;

  public Climb(FileLog log) {
    this.log = log;

    climbMotor1 = new WPI_TalonFX(ClimbConstants.climbMotorPort1);
    climbMotor2 = new WPI_TalonFX(ClimbConstants.climbMotorPort2);
    climbPiston1 = new Solenoid(ClimbConstants.climbPistonPort1);
    climbPiston2 = new Solenoid(ClimbConstants.climbPistonPort2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
