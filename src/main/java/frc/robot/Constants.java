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

    public static final class ShooterConstants {
        public static final int shooter1Port = 30;
        public static final int shooter2Port = 31;
        public static final double shooterDefaultRPM = 3000;
    }

    public static final class FeederConstants {
        public static final int feederPort = 40;
        public static final double feederDefaultRPM = 2000;
    }

    public static final class IntakeConstants {
        public static final int intakePort = 50;
        public static final double intakeDefaultPercentOutput = 0.8;
    }

    public static final class HopperConstants {
        public static final int hopperPort = 51;
        public static final double hopperDefaultPercentOutput = 0.8;
    }

    public static final class DriveConstants{
        public static int leftDriveMotorOne = 10;
        public static int leftDriveMotorTwo = 11;
    
        public static int rightDriveMotorOne = 20;
        public static int rightDriveMotorTwo = 21; 
    
        public static double wheelDiameter = 6.1; // TODO set wheel diameter with actual robot values
        public static double wheelCircumference = 6.1 * Math.PI / 2;
        public static double ticksPerInch = 830.8;
        
        public static int encoderTicksPerRevolution = 2048; // TODO set ticks per rev with actual values

        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;
    
        // from robot characteristics
        public static final double kS = 0.35 * 1; // static gain was
        public static final double kV = 1.665; // velocity gain was 0.551
        public static final double kA = 0.1 * 1; // acceleration gain was 0.000647
    
        public static final double kP = 0.069 * 1; // was 0.069
        public static final double kD = 0.0303;
        public static final double MAX_VOLTAGE = 10.0;
        public static final double TRACK_WIDTH = Units.inchesToMeters(25.35); // was 7.286626058797765
    
        // TODO verify these
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
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
