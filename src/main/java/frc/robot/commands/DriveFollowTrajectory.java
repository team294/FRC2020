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
import frc.robot.utilities.FileLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;

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

  private Pose2d initialPose;

  // Note:  All constants are in ouput units of "percent power" (-1 to +1), not volts!
  private final RamseteController m_ramseteController = new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.kSLinear, DriveConstants.kVLinear ,DriveConstants.kALinear);
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
  private final PIDController m_leftController = new PIDController(DriveConstants.kPLinear, 0, 0);
  private final PIDController m_rightController = new PIDController(DriveConstants.kPLinear, 0, 0);

  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  private final boolean m_useRamsete;
  private final PIDType m_pidType;

  private final DriveTrain driveTrain;
  private final FileLog log;

  public enum PIDType {
    kNone(0),
    kWPILib(1),
    kTalon(2);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    public final int value;

    PIDType(int value) { this.value = value; }
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
   * @param useRamsete      True = use Ramsete controller for feedback to track robot odometery to the trajectory;  False = no trajectory feedback
   * @param pidType         Specify which PIDs to use for feedback to track actual wheel velocities to desired wheel velocities.
   * kNone = no velocity feedback, kWPILib = PID in Rio WPILib software, kTalon (best) = velocity PID on Talon
   * @param driveTrain      The driveTrain subsystem to be controlled.
   * @param log             File for logging
   */
  public DriveFollowTrajectory(Trajectory trajectory, boolean useRamsete, PIDType pidType, DriveTrain driveTrain, FileLog log) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_useRamsete = useRamsete;
    m_pidType = pidType;
    this.driveTrain = driveTrain;
    this.log = log;

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
  * @param log             File for logging
  */
 public DriveFollowTrajectory(Trajectory trajectory, DriveTrain driveTrain, FileLog log) {
   this(trajectory, true, PIDType.kTalon, driveTrain, log);
 }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveFollowTrajectory", "Init");
    initialPose = driveTrain.getPose();

    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter
                * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();

    switch (m_pidType) {
      case kWPILib:
        m_leftController.reset();
        m_rightController.reset();
        break;
      case kTalon:
        driveTrain.setTalonPIDConstants(DriveConstants.kPLinear, DriveConstants.kILinear, DriveConstants.kDLinear, 0);
        driveTrain.resetTalonPIDs();
        break;
      case kNone:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;
    State desiredState = m_trajectory.sample(curTime);

    Pose2d robotPose = driveTrain.getPose().relativeTo(initialPose);
    DifferentialDriveWheelSpeeds robotSpeeds = driveTrain.getWheelSpeeds();

    DifferentialDriveWheelSpeeds targetWheelSpeeds;
    if (m_useRamsete) {
      targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_ramseteController.calculate(robotPose, desiredState));
    } else {
      targetWheelSpeeds =  m_kinematics.toWheelSpeeds( new ChassisSpeeds(
        desiredState.velocityMetersPerSecond, 0.0, 
        desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter) ) ;
    }

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint,
        (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint,
        (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);
        
    double leftOutput = leftFeedforward;
    double rightOutput = rightFeedforward;

    switch (m_pidType) {
      case kNone:
        driveTrain.setLeftMotorOutput(leftOutput);
        driveTrain.setRightMotorOutput(rightOutput);
        break;
      case kWPILib:
        leftOutput += m_leftController.calculate(robotSpeeds.leftMetersPerSecond, leftSpeedSetpoint);
        rightOutput += m_rightController.calculate(robotSpeeds.rightMetersPerSecond, rightSpeedSetpoint);

        driveTrain.setLeftMotorOutput(leftOutput);
        driveTrain.setRightMotorOutput(rightOutput);
        break;
      case kTalon:
        driveTrain.setLeftTalonPIDVelocity(Units.metersToInches(leftSpeedSetpoint), leftOutput);
        driveTrain.setRightTalonPIDVelocity(Units.metersToInches(rightSpeedSetpoint), rightOutput, true);
        break;
    }

    log.writeLog(true, "DriveFollowTrajectory", "Update", 
      "Time", m_timer.get(), 
      "Traj X", desiredState.poseMeters.getTranslation().getX(),
      "Traj Y", desiredState.poseMeters.getTranslation().getY(),
      "Traj ang", desiredState.poseMeters.getRotation().getDegrees(),
      "Robot X", robotPose.getTranslation().getX(),
      "Robot Y", robotPose.getTranslation().getY(),
      "Robot ang", robotPose.getRotation().getDegrees(),
      "Traj VelL", leftSpeedSetpoint,
      "Traj VelR", rightSpeedSetpoint,
      "Robot VelL", robotSpeeds.leftMetersPerSecond,
      "Robot VelR", robotSpeeds.rightMetersPerSecond,
      "Left VFF", leftFeedforward,
      "Left Vout", leftOutput,
      "Right VFF", rightFeedforward,
      "Right Vout", rightOutput
    );

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "DriveFollowTrajectory", "End");
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }
}
