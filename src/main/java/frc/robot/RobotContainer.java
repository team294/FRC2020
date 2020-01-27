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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.triggers.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final FileLog log = new FileLog("A1");
  private final Shooter shooter = new Shooter(log);
  private final Feeder feeder = new Feeder(log);
  private final Intake intake = new Intake();
  private final Hopper hopper = new Hopper();
  private final DriveTrain driveTrain = new DriveTrain(log);
  private final LED led = new LED();

  Joystick xboxController = new Joystick(Constants.OIConstants.xboxControllerPort);
  Joystick leftJoystick = new Joystick(Constants.OIConstants.leftJoystickPort);
  Joystick rightJoystick = new Joystick(Constants.OIConstants.rightJoystickPort);
  // Joystick coPanel = new Joystick(Constants.OIConstants.coPanelPort);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick));
  }

  public void configureShuffleboard() {
    // shooter subsystem
    SmartDashboard.putData("Shooter Manual SetPoint", new ShooterSetPID(shooter));
    SmartDashboard.putData("Shooter STOP", new ShooterSetVoltage(0, shooter));
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 3000);

    // feeder subsystem
    SmartDashboard.putData("Feeder Manual SetPoint", new FeederSetPID(feeder));
    SmartDashboard.putData("Feeder STOP", new FeederSetVoltage(0, feeder));
    SmartDashboard.putData("FeederSetVoltage(5)", new FeederSetVoltage(5, feeder));
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);

    // intake subsystem
    SmartDashboard.putData("IntakeSetPercentOutput(1)", new IntakeSetPercentOutput(1, intake));

    // hopper subsystem
    SmartDashboard.putData("HopperSetPercentOutput(0.8)", new HopperSetPercentOutput(0.8, hopper));

    // led subsystem
    SmartDashboard.putData("LEDSetStrip RED", new LEDSetStrip("Red", led));
    SmartDashboard.putData("LEDSetStrip YELLOW", new LEDSetStrip("Yellow", led));
    SmartDashboard.putData("LEDSetStrip BLUE", new LEDSetStrip("Blue", led));
    SmartDashboard.putData("LEDSetStrip GREEN", new LEDSetStrip("Green", led));
    SmartDashboard.putData("LEDSetStrip OFF", new LEDSetStrip("Red", 0, led));

    // command sequences
    SmartDashboard.putData("ShooterFeederHopperSequence", new ShooterFeederHopperSequence(shooter, feeder, hopper));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  private void configureXboxButtons() {
    JoystickButton[] xb = new JoystickButton[11];
    Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }

    // A = 1, B = 2, X = 3, Y = 4
    xb[1].whenPressed(new ShooterFeederHopperStop(shooter, feeder, hopper));
    xb[2].whenPressed(new ShooterFeederHopperSequence(shooter, feeder, hopper));
    // xb[3].whenPressed(new Wait(0));
    // xb[4].whenPressed(new Wait(0));

    // LB = 5, RB = 6
    // xb[5].whenPressed(new Wait(0));
    // xb[6].whenPressed(new Wait(0));

    // back = 7, start = 8
    // xb[7].whenPressed(new Wait(0));
    // xb[8].whenPressed(new Wait(0));

    // left stick = 9, right stick = 10 (these are buttons when clicked)
    // xb[9].whenPressed(new Wait(0));
    // xb[10].whenPressed(new Wait(0));

    // pov is the d-pad (up, down, left, right)
    // xbPOVUp.whenActive(new Wait(0));
    xbPOVDown.whileActiveOnce(new IntakeSetPercentOutput(intake));
    // xbPOVLeft.whenActive(new Wait(0));
    // xbPOVRight.whenActive(new Wait(0));

    // left and right triggers
    // xbLT.whenActive(new Wait(0));
    // xbRT.whenActive(new Wait(0));
  }

  public void configureJoystickButtons() {
    /*JoystickButton[] left = new JoystickButton[12];
    JoystickButton[] right = new JoystickButton[12];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // joystick trigger
    left[1].whenPressed(new Wait(0));
    right[1].whenPressed(new Wait(0));

    // joystick down button
    left[2].whenPressed(new Wait(0));
    right[2].whenPressed(new Wait(0));

    // joystick up button
    left[3].whenPressed(new Wait(0));
    right[3].whenPressed(new Wait(0));

    // joystick left button
    left[4].whenPressed(new Wait(0));
    right[4].whenPressed(new Wait(0));

    // joystick right button
    left[5].whenPressed(new Wait(0));
    right[5].whenPressed(new Wait(0));*/
  }

  /** CoPanel Layout
   *     
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    /*JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP, from left to right
    coP[1].whenPressed(new Wait(0));
    coP[3].whenPressed(new Wait(0));
    coP[5].whenPressed(new Wait(0));

    // top row DOWN, from left to right
    coP[2].whenPressed(new Wait(0));
    coP[4].whenPressed(new Wait(0));
    coP[6].whenPressed(new Wait(0));

    // top row RED SWITCH
    coP[8].whenPressed(new Wait(0));

    // middle row UP, from left to right
    coP[9].whenPressed(new Wait(0));
    coP[11].whenPressed(new Wait(0));
    coP[13].whenPressed(new Wait(0));

    // middle row DOWN, from left to right
    coP[10].whenPressed(new Wait(0));
    coP[12].whenPressed(new Wait(0));
    coP[14].whenPressed(new Wait(0));

    // middle row UP OR DOWN, fourth button
    coP[7].whenPressed(new Wait(0));

    // bottom row UP, from left to right
    coP[15].whenPressed(new Wait(0));

    // bottom row DOWN, from left to right
    coP[16].whenPressed(new Wait(0));*/
  }

  /**
   * Use this to pass autonomous command to Robot class.
   * @return command to run in autonomous
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
            new Translation2d(4,1),
            new Translation2d(6,1)
        ),
        new Pose2d(10, 0, new Rotation2d(0)),
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
