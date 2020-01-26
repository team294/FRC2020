/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.Constants.DriveConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final FileLog log = new FileLog("A1");
  private final DriveTrain driveTrain = new DriveTrain(log);
  private final Test test = new Test();

  public Joystick leftJoystick = new Joystick(0);
  public Joystick rightJoystick = new Joystick(1);
  public Joystick coPanel = new Joystick(2);
  public Joystick xBoxControler = new Joystick(3);

  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Config shuffleboard
    configureShuffleboard();

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick)); //TODO uncomment after testing
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  private void configureShuffleboard(){
    // buttons for testing turnGyro
    SmartDashboard.putData("Turn90", new DriveTurnGyro(driveTrain, log, 90, 0.25, 0.25));
    SmartDashboard.putData("ZeroGyro", new DriveZeroGyro(driveTrain));
    SmartDashboard.putData("FullSendTurn", new DriveSetPercentOutput(driveTrain, 1, -1)); // to calculate max angular velocity
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return getTestTrajectory();
  }

  
  private DifferentialDriveKinematics getDriveKinematics(double trackWidth) {
    return new DifferentialDriveKinematics(trackWidth);
  }

  public Command getTestTrajectory() {

    driveTrain.zeroLeftEncoder();
    driveTrain.zeroRightEncoder();
    driveTrain.zeroGyroRotation();
    // use starting angle to control trajectory
    double startAngle = driveTrain.getGyroRotation();
    System.out.println(startAngle);

    DifferentialDriveKinematics driveKinematics = getDriveKinematics(DriveConstants.TRACK_WIDTH);

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA), 
        driveKinematics,
        DriveConstants.MAX_VOLTAGE);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(driveKinematics)
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    // Start at the origin facing the +X direction
    // Pass through these two interior waypoints, making an 's' curve path
    // End 3 meters straight ahead of where we started, facing forward
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(2,0)
            // new Translation2d(3,0)
        ),
        new Pose2d(4, 0, new Rotation2d(0)),
        config
    );
    int i = 1;
    for(State s : exampleTrajectory.getStates()) {
      Pose2d pose = s.poseMeters;
      System.out.printf("%f %s %f %n", s.timeSeconds, pose, s.velocityMetersPerSecond);
      i++;
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveTrain::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.kS,DriveConstants.kV,DriveConstants.kA),
        driveKinematics,
        driveTrain::getWheelSpeeds,
        new PIDController(DriveConstants.kP, 0, 0),
        new PIDController(DriveConstants.kP, 0, 0),
        driveTrain::tankDriveVolts,
        driveTrain
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDrive(0.0, 0.0, false));
  }
}
