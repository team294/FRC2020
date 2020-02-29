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
        public static final double temperatureCheck = 40; // in celsius
        public static final double pidErrorTolerance = 200; // in RPM

        // Next row is a DEFAULT VALUE. Change this value in RobotPrefrences for each
        // robot, not in this code!
        public static boolean prototypeBot = false; // true = proto robot, false = competition robot
    }

    public static final class ShooterConstants {
        public static final int canShooterMotorRight = 31; // 30 on competition bot
        public static final int canShooterMotorLeft = 30; // 31 on competition bot
        public static final int pcmShooterHoodPistonIn = 2; // open hood (retract)
        public static final int pcmShooterHoodPistonOut = 3; // close hood (extend)
        public static final int pcmShooterLockPiston = 6; // lock and unlock hood angle
        public static final double shooterDefaultRPM = 2800;
        public static final double shooterDefaultShortRPM = 1400;
        public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for
                                                       // counting power cells)
        public static final int dioPowerCell = 9;
        // public static final double voltageCheck = 7.5; // voltage the shooter will reach if power cell is shot (for counting power cells)
        // public static final double currentCheck = 60; // voltage the shooter will reach if power cell is shot (for counting power cells)
        public static final double hopperPercentCheck = 0.3; // percent output hopper will reach once it is running (for counting power cells)
        
        public static final double[][] distanceFromTargetToRPMTable = {{4,1400},{5,1500},{10,2500},{15,2900},{20,2900},{25,3100},{30,3200}};
        // TODO figure out max distance of robot from target so table includes all necessary values

        public static final double maxSecondsToShoot3balls = 5.0; // max time to wait while shooting 3 balls. use this in commands to timeout
    }

    public static final class FeederConstants {
        public static final int canFeederMotor = 40;
        public static final double feederDefaultRPM = 2000;
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
        public static final double offset = 1.33; // in feet
        //public static final double cameraHeight = 1.625; // in feet, height from floor to lens of mounted camera, 2.104 on protobot
        public static final double cameraHeight = 2.104;
        //public static final double targetHeight = 7.0; // in feet, height to middle of crosshair on target
        public static final double targetHeight = 6.333;
        public static final double cameraAngle = 0.5; // in degrees 26.5 measured but 28 works better?, 14 on proto
        public static final double endDistance = 50; // distance of the "sweet spot" 
        public static final double unlockedHoodMaxDistance = 13.8; // greatest feet away from target that hood needs to be unlocked to make shot
    }

    /**
     * Options to select driving coordinates.
     */
    public enum CoordType {
        kRelative(0),
        kAbsolute(1);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        CoordType(int value) { this.value = value; }
    }

    /**
     * Options to select driving target types.
     */
    public enum TargetType {
        kRelative(0),
        kAbsolute(1),
        kVision(2);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TargetType(int value) { this.value = value; }
    }

    public static final class DriveConstants {

        // *******************************
        // The constants below apply to all robots and are not in RobotPreferences
        // *******************************

        public static final int canLeftDriveMotor1 = 10;
        public static final int canLeftDriveMotor2 = 11;

        public static final int canRightDriveMotor1 = 20;
        public static final int canRightDriveMotor2 = 21;

        public static final double compensationVoltage = 12.0; // voltage compensation on drive motors
        public static final double MAX_VOLTAGE_IN_TRAJECTORY = 10.0;

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;

        // *******************************
        // The constants below are DEFAULT VALUES. Change these value in RobotPrefrences
        // for each robot, not in this code!
        // *******************************

        public static double ticksPerInch = 1103.9; // TODO Practice bot = 830.8, 1103.9 on competition bot

        // public static double wheelDiameterInches = 6.1; //TODO set wheel diameter
        // with actual robot values
        // public static double wheelCircumferenceInches = wheelDiameterInches *
        // Math.PI;
        // public static double encoderTicksPerRevolution = 2048 * 9.47; // Gear ratio =
        // 9.47 on competition bot
        // public static final double kEncoderDistanceInchesPerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        // (wheelDiameterInches * Math.PI) / (double) encoderTicksPerRevolution;
        // 1/ticksPerInch;

        // turnGyro constants
        public static double kMaxAngularVelocity = 1125; // degrees per second TODO calculate on actual 2020 robot
        public static double kMaxAngularAcceleration = 200; // degrees per second per second 200 on competition bot
        public static double kVAngular = 0.001; // 0.000850 on practice bot, 0.001 on competition bot
        public static double kAAngular = 0.0001;  // 0.0001 on practice bot, 0.0001 on competition bot
        public static double kSAngular = 0.0348;   // 0.0568 on practice bot, 0.0348 (try 0.0500) on competition bot
        public static double kPAngular = 0.0005;   // was 0.001
        public static double kDAngular = 0;
        public static double kIAngular = 0;
        public static final double maxSecondsForTurnGyro = 2.0; // max time to wait for turn gyro. use this in commands to timeout

        // DriveStraight constants
        public static double kMaxSpeedMetersPerSecond = 5.22; // 5.0 on practice bot, 5.22 on competition bot
        public static double kMaxAccelerationMetersPerSecondSquared = 3.8; // 3.8 on practice bot, 3.8 on competition bot
        public static double kVLinear = 0.187; // 0.148 on practice bot, 0.187 on competition bot
        public static double kALinear = 0.025; // 0.025 on practice bot, 0.0184 on competition bot (competition cal=0.0184)
        public static double kSLinear = 0.024; // 0.022 on practice bot, 0.024 on competition bot
        public static double kPLinear = 0.100; // 0.100 on practice bot, 0.100 on competition bot
        public static double kILinear = 0; // 0.0 on both bot
        public static double kDLinear = 0; // 0.0 on both bot
        public static double kAngLinear = 0.030; // 0.030 on both bots

        // Trajectory generation constants
        public static double kS = kSLinear * compensationVoltage; 
        public static double kV = kVLinear * compensationVoltage; 
        public static double kA = kALinear * compensationVoltage; 

        public static double TRACK_WIDTH = Units.inchesToMeters(24.93);   // 25.35 on practice bot, 24.93 on competition bot

        public static void updateDerivedConstants() {
            kS = kSLinear * compensationVoltage; 
            kV = kVLinear * compensationVoltage; 
            kA = kALinear * compensationVoltage; 
        }
    }

    public static final class OIConstants {
        public static final int usbXboxController = 0;
        public static final int usbLeftJoystick = 1;
        public static final int usbRightJoystick = 2;
        public static final int usbCoPanel = 3;
    }
}
