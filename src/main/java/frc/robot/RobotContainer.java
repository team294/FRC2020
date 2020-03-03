/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
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
  private final Hopper hopper = new Hopper(log);
  private final Intake intake = new Intake(log);
  private final LED led = new LED();
  private final Feeder feeder = new Feeder(log, tempCheck);
  private final Climb climb = new Climb(log);
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
  public boolean autoUseVision;

  private final Timer disabledDisplayTimer = new Timer();
  private int displayCount = 1;
  
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
    SmartDashboard.putData("Shooter Manual SetPoint", new ShooterSetPID(false, true, shooter, limeLight, led, log));
    SmartDashboard.putData("Shooter STOP", new ShooterSetVoltage(0, shooter, log));
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
    SmartDashboard.putData("Shooter Forward Calibrate", new ShooterSetVoltage(5, shooter, log));

    // shooter distance to RPM test
    SmartDashboard.putData("Shooter Distance SetPoint", new ShooterSetPID(true, true, shooter, limeLight, led, log));
    SmartDashboard.putNumber("Shooter Distance", 5);
    SmartDashboard.putData("Shooter DistToRPM", new ShooterDistToRPM(shooter, log));
    SmartDashboard.putNumber("Shooter RPM from Dist", 0);

    // feeder subsystem
    SmartDashboard.putData("Feeder STOP", new FeederSetVoltage(0, feeder, log));
    SmartDashboard.putData("Feeder Manual SetPoint", new FeederSetPID(feeder, log));
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);
    SmartDashboard.putData("Feeder Forward Calibrate", new FeederSetVoltage(5, feeder, log));

    // intake subsystem
    SmartDashboard.putData("Intake Forward Calibrate", new IntakeSetPercentOutput(0.5, false, intake, log));
    SmartDashboard.putData("Intake Reverse Calibrate", new IntakeSetPercentOutput(-0.5, false, intake, log));
    SmartDashboard.putData("IntakePiston EXTEND", new IntakePistonSetPosition(true, intake, log));
    SmartDashboard.putData("IntakePiston RETRACT", new IntakePistonSetPosition(false, intake, log));
    SmartDashboard.putData("Intake STOP", new IntakeSetPercentOutput(0, true, intake, log));

    // hopper subsystem
    SmartDashboard.putData("Hopper Forward Calibrate", new HopperSetPercentOutput(0.5, false, hopper, log));
    SmartDashboard.putData("Hopper Reverse Calibrate", new HopperSetPercentOutput(-0.5, false, hopper, log));
    SmartDashboard.putData("Hopper STOP", new HopperSetPercentOutput(0, true, hopper, log));

    // led subsystem
    SmartDashboard.putData("LEDSetStrip OFF", new LEDSetStrip("Red", 0, led, log));
    SmartDashboard.putData("LEDRainbow", new LEDSetPattern(LED.rainbowLibrary, 1, 0.5, led, log));

    // climber subsystem
    SmartDashboard.putData("ClimbLeft 0.8%", new ClimbLeftSetPercentOutput(0.8, climb));
    SmartDashboard.putData("ClimbLeft -0.8%", new ClimbLeftSetPercentOutput(-0.8, climb));
    SmartDashboard.putData("ClimbLeft -6 ips", new ClimbLeftSetVelocity(-6, 6, climb));
    SmartDashboard.putData("ClimbRight 0.8%", new ClimbRightSetPercentOutput(0.8, climb));
    SmartDashboard.putData("ClimbRight -0.8%", new ClimbRightSetPercentOutput(-0.8, climb));
    SmartDashboard.putData("ClimbRight -6 ips", new ClimbRightSetVelocity(-6, 6, climb));
    SmartDashboard.putData("ClimbPistons EXTEND", new ClimbPistonsSetPosition(true, climb));
    SmartDashboard.putData("ClimbPistons RETRACT", new ClimbPistonsSetPosition(false, climb));
    
    // limelight subsystem
    SmartDashboard.putData("Limelight Reset Snapshot Count", new LimelightSnapshotCountReset(limeLight, log));
    //SmartDashboard.putData("Limelight Snapshot Test" , new LimeLightSnapshotTest(limeLight)); // uncomment if limelight snapshot-taking has to be tested

    // command sequences
    SmartDashboard.putData("ShootSequence 2800", new ShootSequence(2800, shooter, feeder, hopper, intake, led, log));
    SmartDashboard.putData("ShootSequence DIST", new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log));
    SmartDashboard.putData("ShootSequence STOP", new ShootSequenceStop(shooter, feeder, hopper, intake, led, log));
    SmartDashboard.putData("ShooterHood OPEN", new ShooterHoodPistonSequence(false, false, shooter, log));
    SmartDashboard.putData("ShooterHood CLOSE, LOCK", new ShooterHoodPistonSequence(true, true, shooter, log));
    SmartDashboard.putData("ShooterHood CLOSE, UNLOCK", new ShooterHoodPistonSequence(true, false, shooter, log));

    // buttons for testing drive code, not updating numbers from SmartDashboard
    SmartDashboard.putData("DriveForever", new DriveSetPercentOutput(0.4, 0.4, driveTrain, log));
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
    SmartDashboard.putData("ZeroGyro", new DriveZeroGyro(driveTrain, log));
    SmartDashboard.putData("ZeroEncoders", new DriveZeroEncoders(driveTrain, log));
    SmartDashboard.putData("ZeroOdometry", new DriveResetPose(0, 0, 0, driveTrain, log));
    SmartDashboard.putData("DriveTrajectoryRelative", new DriveFollowTrajectory(CoordType.kRelative, TrajectoryTest.calcTrajectory(log), driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)));
    SmartDashboard.putData("DriveTrajectoryAbsolute", new DriveFollowTrajectory(CoordType.kAbsolute, TrajectoryTest.calcTrajectory(log), driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)));

    // auto selection widget
    autoChooser.setDefaultOption("ShootBackup", AutoSelection.SHOOT_BACKUP);
    autoChooser.addOption("OpponentTrenchPickup", AutoSelection.OPPONENT_TRENCH_PICKUP);
    autoChooser.addOption("ShootForward", AutoSelection.SHOOT_FORWARD);
    autoChooser.addOption("TrussPickup", AutoSelection.TRUSS_PICKUP);
    autoChooser.addOption("OwnTrenchPickup", AutoSelection.OWN_TRENCH_PICKUP);
    SmartDashboard.putData("Autonomous routine", autoChooser);
    SmartDashboard.putNumber("Autonomous delay", 0);
    SmartDashboard.putBoolean("Autonomous use vision", true);

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
    // Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }

    // A = 1, B = 2, X = 3, Y = 4
    xb[1].toggleWhenPressed(new IntakeSequence(intake, log)); // deploy intake and run rollers in
    xb[2].whileHeld(new ShootSequenceSetup(false, shooter, led, log)); // autoline setup with fixed RPM
    xb[2].whenReleased(new ShootSequence(false, shooter, feeder, hopper, intake, led, log)); // shoot with fixed RPM from autoline
    xb[3].whenPressed(new IntakePistonSetPosition(false, intake, log)); // retract intake
    xb[4].whileHeld(new ShootSequenceSetup(true, shooter, led, log)); // trench setup with fixed RPM
    xb[4].whenReleased(new ShootSequence(true, shooter, feeder, hopper, intake, led, log)); // shoot with fixed RPM from trench

    // LB = 5, RB = 6
    xb[5].whileHeld(new ShootSequenceSetup(false, shooter, limeLight, led, log)); // close shot setup
    xb[5].whenReleased(new ShootSequence(shooter, feeder, hopper, intake, limeLight, led, log)); // shooting sequence
    // xb[6].whileHeld(new ShooterSetPID(true, false, shooter, limeLight, led)); // set shooter rpm
    xb[6].whileHeld(new ShootSequenceSetup(true, shooter, limeLight, led, log)); // normal and far shot setup
    xb[6].whenReleased(new ShootSequence(true, shooter, feeder, hopper, intake, limeLight, led, log)); // shooting sequence

    // back = 7, start = 8 
    // xb[7].whenPressed(new IntakeSetPercentOutput(-1 * Constants.IntakeConstants.intakeDefaultPercentOutput, intake)); // run rollers out
    xb[8].toggleWhenPressed(new IntakeSetPercentOutput(-1 * Constants.IntakeConstants.intakeDefaultPercentOutput, false, intake, log)); // run rollers out

    // left stick = 9, right stick = 10 (these are buttons when clicked)
    // xb[9].whenPressed(new Wait(0));
    // xb[10].whenPressed(new Wait(0));

    // pov is the d-pad (up, down, left, right)
    xbPOVUp.whenActive(new ShooterHoodPistonSequence(false, false, shooter, log)); // open the shooter hood
    xbPOVDown.whenActive(new ShooterHoodPistonSequence(true, false, shooter, log)); // close and unlock shooter hood
    xbPOVLeft.whenActive(new ShooterHoodPistonSequence(true, true, shooter, log)); // close and lock shooter hood
    // xbPOVRight.whenActive(new Wait(0));

    // left and right triggers
    // xbLT.whenActive(new ShooterHoodPistonSequence(true, true, shooter)); // close shooter hood and lock angle
    xbRT.whenActive(new ShootSequenceStop(shooter, feeder, hopper, intake, led, log)); // stop motors and set shooter to low rpm
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
    coP[1].whenPressed(new ClimbPistonsSetPosition(true, climb)); // deploy climb pistons
    coP[3].whileHeld(new ClimbLeftSetPercentOutput(0.8, climb)); // manually raise left climb arm
    coP[5].whileHeld(new ClimbRightSetPercentOutput(0.8, climb)); // manually raise right climb arm

    // top row DOWN, from left to right
    coP[2].whenPressed(new ClimbPistonsSetPosition(false, climb)); // retract climb pistons
    coP[4].whileHeld(new ClimbLeftSetPercentOutput(-0.8, climb)); // manually lower left climb arm
    coP[6].whileHeld(new ClimbRightSetPercentOutput(-0.8, climb)); // manually lower right climb arm

    // top row RED SWITCH
    coP[8].whenPressed(new ClimbSetVelocity(ClimbConstants.defaultVelocity, ClimbConstants.targetLowPosition, climb)); // climb lift sequence

    // middle row UP, from left to right
    coP[9].whileHeld(new ClimbSetPercentOutput(0.8, climb));
    coP[11].whenPressed(new ClimbSetVelocity(-1 * ClimbConstants.defaultVelocity, ClimbConstants.targetHighPosition, climb)); // climb grab sequence
    /*coP[13].whenPressed(new Wait(0));*/

    // middle row DOWN, from left to right
    coP[10].whileHeld(new ClimbSetPercentOutput(-0.4, climb));
    /*coP[12].whenPressed(new Wait(0));
    coP[14].whenPressed(new Wait(0));*/

    // middle row UP OR DOWN, fourth button
    coP[7].whenPressed(new ShooterSetVoltage(0, shooter, log)); // stop shooter

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
    if(SmartDashboard.getNumber("Autonomous delay", -9999) == -9999) {
      SmartDashboard.putNumber("Autonomous delay", 0);
    }
    autoUseVision = SmartDashboard.getBoolean("Autonomous use vision", false);
    autoDelay = SmartDashboard.getNumber("Autonomous delay", 0);
    autoDelay = (autoDelay < 0) ? 0 : autoDelay; // make sure autoDelay isn't negative
    autoDelay = (autoDelay > 15) ? 15 : autoDelay; // make sure autoDelay is only active during auto
    return autoSelection.getAutoCommand(autoDelay, autoUseVision, autoChooser.getSelected(), driveTrain, shooter, feeder, hopper, intake, limeLight, log, led);
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
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    disabledDisplayTimer.reset();
    disabledDisplayTimer.start();

    driveTrain.setDriveModeCoast(true);
    limeLight.setSnapshot(false);
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
    if(displayCount > 1) displayCount = 0;
    led.setPattern(LED.teamFlashingColorsLibrary[displayCount], 0.5, 1);

    if(disabledDisplayTimer.advanceIfElapsed(0.25)) {
      displayCount++;
    }
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
