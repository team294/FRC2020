/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.DriveTurnGyro.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.triggers.*;

import static frc.robot.Constants.OIConstants.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final FileLog log = new FileLog("A1");
  private final TemperatureCheck tempCheck = new TemperatureCheck();
  private final Hopper hopper = new Hopper();
  private final Feeder feeder = new Feeder(log, tempCheck);
  private final Intake intake = new Intake();
  private final LED led = new LED();
  // private final Test test = new Test();
  
  private final RobotPreferences robotPrefs = new RobotPreferences();
  private final DriveTrain driveTrain = new DriveTrain(log, tempCheck);
  private final LimeLight limeLight = new LimeLight(log, led, driveTrain);
  private final Shooter shooter = new Shooter(hopper, log, tempCheck, led, limeLight);
  // private final UsbCamera intakeCamera;
  

  Joystick xboxController = new Joystick(xboxControllerPort);
  Joystick leftJoystick = new Joystick(leftJoystickPort);
  Joystick rightJoystick = new Joystick(rightJoystickPort);
  Joystick coPanel = new Joystick(coPanelPort);

  private AutoSelection autoSelection;
  private SendableChooser<Integer> autoChooser = new SendableChooser<>();
  

  private boolean isEnabled = false;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // intakeCamera = CameraServer.getInstance().startAutomaticCapture();

    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick, log));

    autoSelection = new AutoSelection(log); // initialize stuff for auto routines
  }

  /**
   * Use this method to define your Shuffleboard mappings.
   */
  public void configureShuffleboard() {
    // shooter subsystem
    SmartDashboard.putData("Shooter Manual SetPoint", new ShooterSetPID(false, shooter, limeLight, led));
    SmartDashboard.putData("Shooter STOP", new ShooterSetVoltage(0, shooter));
    SmartDashboard.putNumber("Shooter Manual SetPoint RPM", 2800);
    SmartDashboard.putData("Shooter UNLOCK", new ShooterSetLockPiston(true, shooter));
    SmartDashboard.putData("Shooter LOCK", new ShooterSetLockPiston(false, shooter));

    // shooter distance to RPM test
    SmartDashboard.putData("Shooter Distance SetPoint", new ShooterSetPID(true, shooter, limeLight, led));
    SmartDashboard.putNumber("Shooter Distance", 5);
    SmartDashboard.putData("Shooter DistToRPM", new ShooterDistToRPM(shooter));
    SmartDashboard.putNumber("Shooter RPM from Dist", 0);

    // feeder subsystem
    SmartDashboard.putData("Feeder Manual SetPoint", new FeederSetPID(feeder));
    SmartDashboard.putData("Feeder STOP", new FeederSetVoltage(0, feeder));
    SmartDashboard.putData("FeederSetVoltage(5)", new FeederSetVoltage(5, feeder));
    SmartDashboard.putNumber("Feeder Manual SetPoint RPM", 2000);

    // intake subsystem
    SmartDashboard.putData("IntakeSetPercentOutput(1)", new IntakeSetPercentOutput(1, intake));
    SmartDashboard.putData("IntakePiston EXTEND", new IntakePistonSetPosition(true, intake));
    SmartDashboard.putData("IntakePiston RETRACT", new IntakePistonSetPosition(false, intake));
    SmartDashboard.putData("Intake STOP", new IntakeSetPercentOutput(0, intake));

    // hopper subsystem
    SmartDashboard.putData("HopperSetPercentOutput(0.8)", new HopperSetPercentOutput(0.8, hopper));
    SmartDashboard.putData("Hopper STOP", new HopperSetPercentOutput(0, hopper));

    // led subsystem
    SmartDashboard.putData("LEDSetStrip RED", new LEDSetStrip("Red", led));
    SmartDashboard.putData("LEDSetStrip YELLOW", new LEDSetStrip("Yellow", led));
    SmartDashboard.putData("LEDSetStrip BLUE", new LEDSetStrip("Blue", led));
    SmartDashboard.putData("LEDSetStrip GREEN", new LEDSetStrip("Green", led));
    SmartDashboard.putData("LEDSetStrip OFF", new LEDSetStrip("Red", 0, led));
    SmartDashboard.putData("LEDRainbow", new LEDRainbow(1, 0.5, led));

    // command sequences
    SmartDashboard.putData("ShooterFeederHopperSequence", new ShooterFeederHopperSequence(2800, shooter, feeder, hopper, intake, led));
    SmartDashboard.putData("ShooterFeederHopperIntakeStop", new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led));
    SmartDashboard.putData("ShooterHood OPEN", new ShooterHoodPistonSequence(true, shooter));
    SmartDashboard.putData("ShooterHood CLOSE", new ShooterHoodPistonSequence(false, shooter));

    // buttons for testing turnGyro
    SmartDashboard.putData("ZeroGyro", new DriveZeroGyro(driveTrain));
    SmartDashboard.putData("FullSendTurn", new DriveSetPercentOutput(1, 1, driveTrain)); // to calculate max angular velocity
    SmartDashboard.putData("DriveStraight", new DriveStraight(3, 0.5, 1.0, true, driveTrain, log));
    SmartDashboard.putData("DriveForever", new DriveSetPercentOutput(0.4, 0.4, driveTrain));
    SmartDashboard.putData("TurnVision", new DriveTurnGyro(TargetType.kVision, 0, 0.04, 1.0, 0.5, driveTrain, limeLight, log));
    SmartDashboard.putData("TurnRelative", new DriveTurnGyro(TargetType.kRelative, 180, 0.08, 1.0, 1, driveTrain, limeLight, log));
    SmartDashboard.putData("TurnAbsolute", new DriveTurnGyro(TargetType.kRelative, 180, 0.08, 1.0, 1, driveTrain, limeLight, log));
    SmartDashboard.putData("DriveTrajectory", new DriveFollowTrajectory(TrajectoryTest.calcTrajectory(log), driveTrain, log)
        .andThen(() -> driveTrain.tankDrive(0.0, 0.0, false)));

    // auto selection widget
    autoChooser.setDefaultOption("TrenchStartingCenter", AutoSelection.TRENCH_FROM_CENTER);
    autoChooser.addOption("TrenchStartingRight", AutoSelection.TRENCH_FROM_RIGHT);
    autoChooser.addOption("ShootBackup", AutoSelection.SHOOT_BACKUP);
    autoChooser.addOption("TrussPickup", AutoSelection.TRUSS_PICKUP);
    autoChooser.addOption("OwnTrenchPickup", AutoSelection.OWN_TRENCH_PICKUP);
    SmartDashboard.putData("Autonomous routine", autoChooser);

    // Vision Testing
    SmartDashboard.putData("Vision Zero Drive", new DriveZeroEncoders(driveTrain)); // TODO to be deleted after testing vision distance
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
    // Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    // Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }

    // A = 1, B = 2, X = 3, Y = 4
    xb[1].whenPressed(new ShooterHoodPistonSequence(false, shooter)); // open shooter hood
    // xb[2].whenPressed(new ShooterFeederHopperSequence(false, shooter, feeder, hopper, intake, led));
    // xb[3].whenPressed(new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led));
    xb[4].whenPressed(new ShooterHoodPistonSequence(true, shooter)); // close shooter hood

    // LB = 5, RB = 6
    // xb[5].whenPressed(new Wait(0));
    xb[6].whileHeld(new ShooterSetPID(true, shooter, limeLight, led)); // set shooter rpm TODO change to use distance
    xb[6].whenReleased(new ShooterFeederHopperSequence(true, shooter, feeder, hopper, intake, limeLight, led)); // shooting sequence TODO change to use distance

    // back = 7, start = 8
    // xb[7].whenPressed(new Wait(0));
    // xb[8].whenPressed(new Wait(0));

    // left stick = 9, right stick = 10 (these are buttons when clicked)
    // xb[9].whenPressed(new Wait(0));
    // xb[10].whenPressed(new Wait(0));

    // pov is the d-pad (up, down, left, right)
    xbPOVUp.whenActive(new IntakePistonSetPosition(false, intake)); // retract intake
    xbPOVDown.whileActiveOnce(new IntakeSequence(intake)); // deploy intake and run rollers in
    xbPOVLeft.whenActive(new IntakeSetPercentOutput(-1 * Constants.IntakeConstants.intakeDefaultPercentOutput, intake)); // run rollers out
    // xbPOVRight.whenActive(new Wait(0));

    // left and right triggers
    // xbLT.whenActive(new Wait(0));
    xbRT.whenActive(new ShooterFeederHopperIntakeStop(shooter, feeder, hopper, intake, led)); // stop motors and set shooter to low rpm
  }

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
    right[2].whenHeld(new DriveTurnGyro(TargetType.kVision, 0, 0.04, 1.0, 1, driveTrain, limeLight, log)); // turn gyro with vision
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
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP, from left to right
    /*coP[1].whenPressed(new ClimbPistonSetPosition(true)); // deploy climb pistons
    coP[3].whenPressed(new ClimbLeftSetVoltage(1)); // raise left climb arm
    coP[5].whenPressed(new ClimbRightSetVoltage(1)); // raise right climb arm

    // top row DOWN, from left to right
    coP[2].whenPressed(new ClimbPistonSetPosition(false)); // retract climb pistons
    coP[4].whenPressed(new ClimbLeftSetVoltage(-1)); // lower left climb arm
    coP[6].whenPressed(new ClimbRightSetVoltage(-1)); // lower right climb arm

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
	 * Set xbox controller rumble percent.
	 * @param percentRumble percent rumble (0 to 1)
	 */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);
  }

  /**
   * Use this to pass autonomous command to Robot class.
   * @return command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(autoChooser.getSelected(), driveTrain, shooter, feeder, hopper, intake, limeLight, log, led);
  }

  /**
   * Method called when robot is initialized
   */
  public void robotInit() {
  }

  /**
   * Method called once every scheduler cycle, regardless of
   * whether robot is in auto/teleop/disabled mode
   */
  public void robotPeriodic() {
    log.advanceLogRotation();
  }

  /**
   * Method called robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Mode Init");
    isEnabled = false;
    shooter.setPowerCellsShot(0);
    driveTrain.setDriveModeCoast(true);
    led.setStrip("Green", 1);
    shooter.setShooterPID(0);
    hopper.hopperSetPercentOutput(0);
    feeder.setFeederPID(0);
    intake.intakeSetPercentOutput(0);
  }

  /**
   * Method called once every scheduler cycle when robot is disabled
   */
  public void disabledPeriodic() {
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");
    led.setStrip("Purple", 1);
    driveTrain.startAutoTimer();
    driveTrain.setDriveModeCoast(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.

    shooter.setShooterPID(1200);
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
    isEnabled = true;
    driveTrain.setDriveModeCoast(false);
    shooter.setShooterPID(1200);
  }

  public boolean getEnabled(){
    return isEnabled;
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled
   */

  public void teleopPeriodic() {
  }
}
