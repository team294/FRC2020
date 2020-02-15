/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {
        // Global constants go here

        // Next row is a DEFAULT VALUE.  Change this value in RobotPrefrences for each robot, not in this code!
        public static boolean prototypeBot = false;     // true = proto robot, false = competition robot
    }

    public static final class ShooterConstants {
        public static final int canShooterMotorRight = 31; // 30 on competition bot
        public static final int canShooterMotorLeft = 30; // 31 on competition bot
        public static final int pcmShooterHoodPiston = 2;
        public static final int pcmShooterLockPiston = 3;
        public static final double shooterDefaultRPM = 2800;
        public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for counting power cells)
        public static final double hopperPercentCheck = 0.3; // percent output hopper will reach once it is running (for counting power cells)
        public static final double temperatureCheck = 40; // in celsius TODO this doesn't need to be in every subsystem unless it will have different values??
    }

    public static final class FeederConstants {
        public static final int canFeederMotor = 40;
        public static final double feederDefaultRPM = 2000;
        public static final double temperatureCheck = 40; // in celsius
    }

    public static final class IntakeConstants {
        public static final int canIntakeMotor = 50;
        public static final double intakeDefaultPercentOutput = 0.8;
    }

    public static final class HopperConstants {
        public static final int canHopperMotor = 51;
        public static final double hopperDefaultPercentOutput = 0.8;
    }

    public static final class LimeLightConstants {
        public static final double angleMultiplier = 1.064;
    }

    public static final class DriveConstants {
        public static int canLeftDriveMotor1 = 10;
        public static int canLeftDriveMotor2 = 11;
    
        public static int canRightDriveMotor1 = 20;
        public static int canRightDriveMotor2 = 21;

        public static final double temperatureCheck = 40; // in celsius
    
        public static double wheelDiameterInches = 6.1; //TODO set wheel diameter with actual robot values
        public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;
        public static double ticksPerInch = 830.8;   // Measured with PracticeBot gearbox between wheels and the Falcons
        
        public static int encoderTicksPerRevolution = 2048; //TODO set ticks per rev with actual values
        public static final double kEncoderDistanceInchesPerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            // (wheelDiameterInches * Math.PI) / (double) encoderTicksPerRevolution;
            1/ticksPerInch;

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;
    
        // from robot characteristics
        public static final double kS = 0.35; // static gain was
        public static final double kV = 1.665; // velocity gain was 0.551
        public static final double kA = 0.1; // acceleration gain was 0.000647
    
        public static final double kP = 0.069 * 1; // was 0.069
        public static final double kD = 0.0303;
        public static final double MAX_VOLTAGE = 10.0;
        public static final double TRACK_WIDTH = Units.inchesToMeters(25.35); // was 7.286626058797765
    
        // turnGyro constants
        public static final double kMaxAngularVelocity = 1125; // degrees per second TODO calculate on actual 2020 robot
        public static final double kMaxAngularAcceleration = 400; // degrees per second per second (was 200%)
        public static final double kVAngular = 0.000850; // was 0.000838  then 0.000943
        public static final double kAAngular = 0.0001;  // was 0.0003
        public static final double kSAngular = 0.0568;   // was 0.0568
        public static final double kPAngular = 0.0005;   // was 0.001
        public static final double kDAngular = 0;
        public static final double kIAngular = 0;

        // verify these
        public static final double kMaxSpeedMetersPerSecond = 5.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.8; // was 0.352
        public static final double kVLinear = 0.148; // was 0.09
        public static final double kALinear = 0.0293;  // was 0.07
        public static final double kSLinear = 0.022; // 0.11 for working on a drive base was 0.055
    }

    public static final class OIConstants {
        public static final int xboxControllerPort = 0;
        public static final int leftJoystickPort = 1;
        public static final int rightJoystickPort = 2;
        public static final int coPanelPort = 3;
  
        /*public enum Button {
            kBumperLeft(5),
            kBumperRight(6),
            kStickLeft(9),
            kStickRight(10),
            kA(1),
            kB(2),
            kX(3),
            kY(4),
            kBack(7),
            kStart(8);
    
            @SuppressWarnings({"MemberName", "PMD.SingularField"})
            public final int value;

            Button(int value) { this.value = value; }
        }*/
    }
}
