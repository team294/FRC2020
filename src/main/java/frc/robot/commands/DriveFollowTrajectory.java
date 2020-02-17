/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class DriveFollowTrajectory extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final RamseteController m_ramseteController = new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kS,DriveConstants.kV,DriveConstants.kA);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
  private final PIDController m_leftController = new PIDController(DriveConstants.kP, 0, 0);
  private final PIDController m_rightController = new PIDController(DriveConstants.kP, 0, 0);

  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private final boolean m_useRamsete;
  private final boolean m_usePID;

  private final DriveTrain driveTrain;

  /**
   * Constructs a new command that, when executed, will follow the provided trajectory.
   * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
   * representing units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this
   * is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory      The trajectory to follow.
   * @param useRamsete      True = use Ramsete controller for feedback to track robot odometery to the trajectory;  False = no trajectory feedback
   * @param usePID          True = use PIDs for feedback to track actual wheel velocities to desired wheel velocities;  False = not velocity feedback
   * @param driveTrain      The driveTrain subsystem to be controlled.
   */
  public DriveFollowTrajectory(Trajectory trajectory, boolean useRamsete, boolean usePID, DriveTrain driveTrain) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_useRamsete = useRamsete;
    m_usePID = usePID;
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);
  }  
  
  /**
  * Constructs a new command that, when executed, will follow the provided trajectory.
  * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
  * representing units of volts.
  *
  * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
  * this
  * is left to the user, since it is not appropriate for paths with nonstationary endstates.
  *
  * @param trajectory      The trajectory to follow.
  * @param driveTrain      The driveTrain subsystem to be controlled.
  */
 public DriveFollowTrajectory(Trajectory trajectory, DriveTrain driveTrain) {
   this(trajectory, true, true, driveTrain);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter
                * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var robotPose = driveTrain.getPose();
    var desiredState = m_trajectory.sample(curTime);

    DifferentialDriveWheelSpeeds targetWheelSpeeds;
    if (m_useRamsete) {
      targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_ramseteController.calculate(robotPose, desiredState));
    } else {
      targetWheelSpeeds =  m_kinematics.toWheelSpeeds( new ChassisSpeeds(
        desiredState.velocityMetersPerSecond, 0.0, 
        desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter) ) ;
    }

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_feedforward.calculate(leftSpeedSetpoint,
              (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          m_feedforward.calculate(rightSpeedSetpoint,
              (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput = leftFeedforward
          + m_leftController.calculate(driveTrain.getWheelSpeeds().leftMetersPerSecond,
          leftSpeedSetpoint);

      rightOutput = rightFeedforward
          + m_rightController.calculate(driveTrain.getWheelSpeeds().rightMetersPerSecond,
          rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    driveTrain.tankDriveVolts(leftOutput, rightOutput);

    // log.writeLog(true, "TankDriveVolts", "Update", 
    //   "Time", autoTimer.get(), 
    //   "L Meters", Units.inchesToMeters(getLeftEncoderInches()),
    //   "R Meters", Units.inchesToMeters(getRightEncoderInches()), 
    //   "L Velocity", Units.inchesToMeters(getLeftEncoderVelocity()), 
    //   "R Velocity", Units.inchesToMeters(-getRightEncoderVelocity()), 
    //   "L Volts", leftVolts, 
    //   "R Volts", rightVolts, 
    //   "Gyro", getGyroRotation(), "Pose Angle", getPose().getRotation().getDegrees());

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }
}
