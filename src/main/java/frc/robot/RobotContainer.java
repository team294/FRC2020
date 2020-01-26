/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Shooter shooter = new Shooter();
  private final Feeder feeder = new Feeder();
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();

  XboxController xBoxController = new XboxController(Constants.OIConstants.xboxControllerPort);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(xBoxController, Button.kX.value).whenPressed(new Wait(0));
    new JoystickButton(xBoxController, Button.kY.value).whenPressed(new Wait(0));
    new JoystickButton(xBoxController, Button.kB.value).whenPressed(new Wait(0));
    new JoystickButton(xBoxController, Button.kA.value).whenPressed(new Wait(0));
  }

  public void configureShuffleboard() {
    // shooter subsystem
    SmartDashboard.putData("Shooter Manual SetPoint", new ShooterSetPID(shooter));
    SmartDashboard.putData("Shooter STOP", new ShooterSetVoltage(0, shooter));
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 3000);

    // intake subsystem
    SmartDashboard.putData("IntakeSetPercentOutput(1)", new IntakeSetPercentOutput(1, intake));

    // hopper subsystem
    SmartDashboard.putData("HopperSetPercentOutput(0.8)", new HopperSetPercentOutput(0.8, hopper));

    // feeder subsystem
    SmartDashboard.putData("Feeder Manual SetPoint", new FeederSetPID(feeder));
    SmartDashboard.putData("Feeder STOP", new FeederSetVoltage(0, feeder));
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 1000);

    // command sequences
    SmartDashboard.putData("ShooterFeederHopperSequence", new ShooterFeederHopperSequence(shooter, feeder, hopper));
  }

  /**
   * Use this method to pass autonomous command to Robot class.
   * @return command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Wait(0);
  }
}
