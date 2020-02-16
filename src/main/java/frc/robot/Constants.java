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
        public static final int dioPowerCell = 9;
       // public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for counting power cells)
       // public static final double currentCheck = 60; // voltage the shooter will reach if power cell is shot (for counting power cells)
        public static final double hopperPercentCheck = 0.3; // percent output hopper will reach once it is running (for counting power cells)
        public static final double temperatureCheck = 40; // in celsius TODO this doesn't need to be in every subsystem unless it will have different values??
        
        public static final double[][] distanceFromTargetToRPMTable = {{5,1200},{10,2000},{15,2250},{20,2500},{25,2750},{30,3000}};
        // TODO figure out max distance of robot from target so table includes all necessary values
    }

    public static final class FeederConstants {
        public static final int canFeederMotor = 40;
        public static final double feederDefaultRPM = 2000;
        public static final double temperatureCheck = 40; // in celsius
    }

    public static final class IntakeConstants {
        public static final int canIntakeMotor = 50;
        public static final int pcmIntakePistonIn = 0;
        public static final int pcmIntakePistonOut = 1;
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

        // *******************************
        // The constants below apply to all robots and are not in RobotPreferences
        // *******************************

        public static final int canLeftDriveMotor1 = 10;
        public static final int canLeftDriveMotor2 = 11;
    
        public static final int canRightDriveMotor1 = 20;
        public static final int canRightDriveMotor2 = 21;

        public static final double temperatureCheck = 40; // in celsius

        public static final double compensationVoltage = 12.0; // voltage compensation on drive motors
        public static final double MAX_VOLTAGE_IN_TRAJECTORY = 10.0;

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;

        // *******************************
        // The constants below are DEFAULT VALUES.  Change these value in RobotPrefrences for each robot, not in this code!
        // *******************************

        public static double ticksPerInch = 830.8;   //TODO Practice bot = 830.8, 1103.9 on competition bot
        
        // public static double wheelDiameterInches = 6.1; //TODO set wheel diameter with actual robot values
        // public static double wheelCircumferenceInches = wheelDiameterInches * Math.PI;       
        // public static double encoderTicksPerRevolution = 2048 * 9.47; // Gear ratio = 9.47 on competition bot
        // public static final double kEncoderDistanceInchesPerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            // (wheelDiameterInches * Math.PI) / (double) encoderTicksPerRevolution;
            // 1/ticksPerInch;

        // turnGyro constants
        public static double kMaxAngularVelocity = 1125; // degrees per second TODO calculate on actual 2020 robot
        public static double kMaxAngularAcceleration = 400; // degrees per second per second (was 200%)
        public static double kVAngular = 0.000850; // was 0.000838  then 0.000943
        public static double kAAngular = 0.0001;  // was 0.0003
        public static double kSAngular = 0.0568;   // was 0.0568
        public static double kPAngular = 0.0005;   // was 0.001
        public static double kDAngular = 0;
        public static double kIAngular = 0;

        // verify these
        public static double kMaxSpeedMetersPerSecond = 5.0;  // 5.0 on practice bot, 5.22 on competition bot
        public static double kMaxAccelerationMetersPerSecondSquared = 3.8; // 3.8 on practice bot, 20.0 on competition bot
        public static double kVLinear = 0.148; // 0.148 on practice bot, 0.187 on competition bot
        public static double kALinear = 0.050;  // 0.025 on practice bot, 0.0184 on competition bot
        public static double kSLinear = 0.022; // 0.022 on practice bot, 0.024 on competition bot

        public static double kPLinear = 0.060;  //0.100 on practice bot
        public static double kILinear = 0;  //0.0 on practice bot
        public static double kDLinear = 0;  //0.0 on practice bot

        // from robot characteristics
        public static double kS = kSLinear * 12; 
        public static double kV = kVLinear * 12; 
        public static double kA = kALinear * 12; 

        public static double kP = kPLinear * 12;  
        public static double kI = kILinear * 12;  
        public static double kD = kDLinear * 12;  

        public static double TRACK_WIDTH = Units.inchesToMeters(25.35); 

        public static void updateDerivedConstants() {
            kS = kSLinear * 12; 
            kV = kVLinear * 12; 
            kA = kALinear * 12; 
    
            kP = kPLinear * 12;  
            kI = kILinear * 12;  
            kD = kDLinear * 12;  
        }
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
