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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.triggers.*;

import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.DriveConstants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final FileLog log = new FileLog("A1");
  private final TemperatureCheck tempCheck = new TemperatureCheck();
  private final LED led = new LED();
  
  private final Hopper hopper = new Hopper();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder(log, tempCheck);
  private final Shooter shooter = new Shooter(hopper, log, tempCheck, led);
  private final DriveTrain driveTrain = new DriveTrain(log, tempCheck);
  private final LimeLight limeLight = new LimeLight(log, led, driveTrain);

  Joystick xboxController = new Joystick(usbXboxController);
  Joystick leftJoystick = new Joystick(usbLeftJoystick);
  Joystick rightJoystick = new Joystick(usbRightJoystick);
  Joystick coPanel = new Joystick(usbCoPanel);

  private AutoSelection autoSelection;
  private SendableChooser<Integer> autoChooser = new SendableChooser<>();
  public double autoDelay;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick, log));

    autoSelection = new AutoSelection(log); // initialize auto selection widget
  }

  /**
   * Define Shuffleboard mappings.
   */
  public void configureShuffleboard() {
    // shooter subsystem
    SmartDashboard.putData("Shooter Manual SetPoint", new ShooterSetPID(false, shooter, limeLight, led));
    SmartDashboard.putData("Shooter STOP", new ShooterSetVoltage(0, shooter));
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
    SmartDashboard.putData("Shooter Forward Calibrate", new ShooterSetVoltage(5, shooter));

    // shooter distance to RPM test
    SmartDashboard.putData("Shooter Distance SetPoint", new ShooterSetPID(true, shooter, limeLight, led));
    SmartDashboard.putNumber("Shooter Distance", 5);
    SmartDashboard.putData("Shooter DistToRPM", new ShooterDistToRPM(shooter));
    SmartDashboard.putNumber("Shooter RPM from Dist", 0);

    // feeder subsystem
    SmartDashboard.putData("Feeder STOP", new FeederSetVoltage(0, feeder));
    SmartDashboard.putData("Feeder Manual SetPoint", new FeederSetPID(feeder));
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);
    SmartDashboard.putData("Feeder Forward Calibrate", new FeederSetVoltage(5, feeder));

    // intake subsystem
    SmartDashboard.putData("Intake Forward Calibrate", new IntakeSetPercentOutput(0.5, intake));
    SmartDashboard.putData("Intake Reverse Calibrate", new IntakeSetPercentOutput(-0.5, intake));
    SmartDashboard.putData("IntakePiston EXTEND", new IntakePistonSetPosition(true, intake));
    SmartDashboard.putData("IntakePiston RETRACT", new IntakePistonSetPosition(false, intake));
    SmartDashboard.putData("Intake STOP", new IntakeSetPercentOutput(0, intake));

    // hopper subsystem
    SmartDashboard.putData("Hopper Forward Calibrate", new HopperSetPercentOutput(0.5, false, hopper));
    SmartDashboard.putData("Hopper Reverse Calibrate", new HopperSetPercentOutput(-0.5, false, hopper));
    SmartDashboard.putData("Hopper STOP", new HopperSetPercentOutput(0, false, hopper));

    // led subsystem
    SmartDashboard.putData("LEDSetStrip OFF", new LEDSetStrip("Red", 0, led));
    SmartDashboard.putData("LEDRainbow", new LEDRainbow(1, 0.5, led));

    // command sequences
    SmartDashboard.putData("ShootSequence 2800", new ShootSequence(2800, shooter, feeder, hopper, intake, led));
    SmartDashboard.putData("ShootSequence DIST", new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led));
    SmartDashboard.putData("ShootSequence STOP", new ShootSequenceStop(shooter, feeder, hopper, intake, led));
    SmartDashboard.putData("ShooterHood OPEN", new ShooterHoodPistonSequence(true, false, shooter));
    SmartDashboard.putData("ShooterHood CLOSE, LOCK", new ShooterHoodPistonSequence(false, true, shooter));
    SmartDashboard.putData("ShooterHood CLOSE, UNLOCK", new ShooterHoodPistonSequence(false, false, shooter));

    // buttons for testing drive code, not updating numbers from SmartDashboard
    SmartDashboard.putData("DriveForever", new DriveSetPercentOutput(0.4, 0.4, driveTrain));
    SmartDashboard.putData("DriveStraightRel", new DriveStraight(3, TargetType.kRelative, 0.0, 2.66, 3.8, true, driveTrain, limeLight, log));
    SmartDashboard.putData("DriveStraightAbs", new DriveStraight(3, TargetType.kAbsolute, 0.0, 2.66, 3.8, true, driveTrain, limeLight, log));
    SmartDashboard.putData("DriveStraightVis", new DriveStraight(3, TargetType.kVision, 0.0, 2.66, 3.8, true, driveTrain, limeLight, log));
    SmartDashboard.putData("TurnVision", new DriveTurnGyro(TargetType.kVision, 0, 45, 200, 0.5, driveTrain, limeLight, log));
    SmartDashboard.putData("TurnRelative", new DriveTurnGyro(TargetType.kRelative, 90, 90, 200, 1, driveTrain, limeLight, log));
    SmartDashboard.putData("TurnAbsolute", new DriveTurnGyro(TargetType.kAbsolute, 90, 90, 200, 1, driveTrain, limeLight, log));

    // drive profile calibration buttons
    SmartDashboard.putData("TurnGyroManual", new DriveTurnGyro(TargetType.kRelative, true, driveTrain, limeLight, log));
    SmartDashboard.putNumber("TurnGyro Manual Target Ang", 90);
    SmartDashboard.putNumber("TurnGyro Manual MaxVel", kMaxAngularVelocity*0.08);
    SmartDashboard.putNumber("TurnGyro Manual MaxAccel", kMaxAngularAcceleration);
    SmartDashboard.putNumber("TurnGyro Manual Tolerance", 2);
    SmartDashboard.putData("DriveStraightManual", new DriveStraight(TargetType.kRelative, true, driveTrain, limeLight, log));
    SmartDashboard.putNumber("DriveStraight Manual Target Dist", 2);
    SmartDashboard.putNumber("DriveStraight Manual Angle", 0);
    SmartDashboard.putNumber("DriveStraight Manual MaxVel", kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("DriveStraight Manual MaxAccel", kMaxAccelerationMetersPerSecondSquared);
    
    // Testing for autos and trajectories
    SmartDashboard.putData("ZeroGyro", new DriveZeroGyro(driveTrain));
    SmartDashboard.putData("ZeroEncoders", new DriveZeroEncoders(driveTrain));
    SmartDashboard.putData("ZeroOdometry", new DriveResetPose(0, 0, 0, driveTrain));
    SmartDashboard.putData("DriveTrajectoryRelative", new DriveFollowTrajectory(CoordType.kRelative, TrajectoryTest.calcTrajectory(log), driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)));
    SmartDashboard.putData("DriveTrajectoryAbsolute", new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryTest.calcTrajectory(log), driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)));

    // auto selection widget
    autoChooser.setDefaultOption("TrenchStartingCenter", AutoSelection.TRENCH_FROM_CENTER);
    autoChooser.addOption("TrenchStartingRight", AutoSelection.TRENCH_FROM_RIGHT);
    autoChooser.addOption("ShootBackup", AutoSelection.SHOOT_BACKUP);
    autoChooser.addOption("TrussPickup", AutoSelection.TRUSS_PICKUP);
    autoChooser.addOption("OwnTrenchPickup", AutoSelection.OWN_TRENCH_PICKUP);
    SmartDashboard.putData("Autonomous routine", autoChooser);
    SmartDashboard.putNumber("Autonomous delay", 0);

    // display sticky faults
    RobotPreferences.showStickyFaults();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));
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

  /**
   * Define Xbox controller mappings.
   */
  private void configureXboxButtons() {
    JoystickButton[] xb = new JoystickButton[11];
    Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    // Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }

    // A = 1, B = 2, X = 3, Y = 4
    // xb[1].whenPressed(new Wait(0)));
    // xb[2].whenPressed(new Wait(0)));
    // xb[3].whenPressed(new Wait(0)));
    // xb[4].whenPressed(new Wait(0)));

    // LB = 5, RB = 6
    xb[5].whenPressed(new ShooterHoodPistonSequence(false, false, shooter)); // open shooter hood
    xb[6].whileHeld(new ShooterSetPID(true, shooter, limeLight, led)); // set shooter rpm
    xb[6].whenReleased(new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led)); // shooting sequence

    // back = 7, start = 8
    xb[7].whenPressed(new ShooterHoodPistonSequence(true, false, shooter)); // close shooter hood and do not lock angle
    // xb[8].whenPressed(new Wait(0));

    // left stick = 9, right stick = 10 (these are buttons when clicked)
    // xb[9].whenPressed(new Wait(0));
    // xb[10].whenPressed(new Wait(0));

    // pov is the d-pad (up, down, left, right)
    xbPOVUp.whenActive(new IntakePistonSetPosition(false, intake)); // retract intake
    xbPOVDown.whileActiveOnce(new IntakeSequence(intake)); // deploy intake and run rollers in
    xbPOVLeft.whileActiveOnce(new IntakeSetPercentOutput(-1 * Constants.IntakeConstants.intakeDefaultPercentOutput, intake)); // run rollers out
    // xbPOVRight.whenActive(new Wait(0));

    // left and right triggers
    xbLT.whenActive(new ShooterHoodPistonSequence(true, true, shooter)); // close shooter hood and lock angle
    xbRT.whenActive(new ShootSequenceStop(shooter, feeder, hopper, intake, led)); // stop motors and set shooter to low rpm
  }

  /**
   * Define Joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // joystick left button
    // left[1].whenPressed(new Wait(0));
    // right[1].whenPressed(new Wait(0));

    // joystick right button
    // left[2].whenPressed(new Wait(0));
    right[2].whenHeld(new DriveTurnGyro(TargetType.kVision, 0, 450, 200, 1, driveTrain, limeLight, log)); // turn gyro with vision
  }

  /** 
   * Define Copanel button mappings.
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
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP, from left to right
    /*coP[1].whenPressed(new ClimbPistonSetPosition(true)); // deploy climb pistons
    coP[3].whenPressed(new ClimbLeftSetPercentOutput(0.5)); // manually raise left climb arm
    coP[5].whenPressed(new ClimbRightSetPercentOutput(0.5)); // manually raise right climb arm

    // top row DOWN, from left to right
    coP[2].whenPressed(new ClimbPistonSetPosition(false)); // retract climb pistons
    coP[4].whenPressed(new ClimbLeftSetPercentOutput(-0.5)); // manually lower left climb arm
    coP[6].whenPressed(new ClimbRightSetPercentOutput(-0.5)); // manually lower right climb arm

    // top row RED SWITCH
    coP[8].whenPressed(new ClimbSequence()); // climb sequence

    // middle row UP, from left to right
    coP[9].whenPressed(new Wait(0));
    coP[11].whenPressed(new Wait(0));
    coP[13].whenPressed(new Wait(0));

    // middle row DOWN, from left to right
    coP[10].whenPressed(new Wait(0));
    coP[12].whenPressed(new Wait(0));
    coP[14].whenPressed(new Wait(0));*/

    // middle row UP OR DOWN, fourth button
    coP[7].whenPressed(new ShooterSetVoltage(0, shooter)); // stop shooter

    // bottom row UP, from left to right
    /*coP[15].whenPressed(new Wait(0));

    // bottom row DOWN, from left to right
    coP[16].whenPressed(new Wait(0));*/
  }

  /**
	 * Set Xbox Controller rumble percent.
	 * @param percentRumble percent rumble (0 to 1)
	 */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);
  }

  /**
   * @return command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // get value of delay for beginning of auto from Shuffleboard
    // TODO actually implement autoDelay variable in auto
    if(SmartDashboard.getNumber("Autonomous delay", -9999) == -9999) {
      SmartDashboard.putNumber("Autonomous delay", 0);
    }
    autoDelay = SmartDashboard.getNumber("Autonomous delay", 0);
    autoDelay = (autoDelay < 0) ? 0 : autoDelay; // make sure autoDelay isn't negative
    autoDelay = (autoDelay > 15) ? 15 : autoDelay; // make sure autoDelay is only active during auto
    return autoSelection.getAutoCommand(autoChooser.getSelected(), driveTrain, shooter, feeder, hopper, intake, limeLight, log, led);
  }

  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }
  }

  /**
   * Method called once every scheduler cycle, regardless of
   * whether robot is in auto/teleop/disabled mode.
   */
  public void robotPeriodic() {
    log.advanceLogRotation();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Mode Init");
    led.setStrip("Green", 1);

    driveTrain.setDriveModeCoast(true);
    shooter.setPowerCellsShot(0);
    shooter.setShooterVoltage(0);
    hopper.hopperSetPercentOutput(0);
    feeder.feederSetVoltage(0);
    intake.intakeSetPercentOutput(0);
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");
    led.setStrip("Purple", 1);
    driveTrain.setDriveModeCoast(false);
    shooter.setShooterPID(1200);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");
    led.setStrip("Red", 1);

    driveTrain.setDriveModeCoast(false);
    shooter.setShooterPID(1200);
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
  }
}
