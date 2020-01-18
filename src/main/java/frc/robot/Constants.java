/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static int leftDriveMotorOne = 10;
        public static int leftDriveMotorTwo = 11;
    
        public static int rightDriveMotorOne = 20;
        public static int rightDriveMotorTwo = 21; 
    
        public static double wheelDiameter = 6.1; //TODO set wheel diameter with actual robot values
        public static double wheelCircumference = 6.1 * Math.PI / 2;
        
        public static int encoderTicksPerRevolution = 2048; //TODO set ticks per rev with actual values

        public static final int MOTOR_LEFT_MASTER = 10;
        public static final int MOTOR_LEFT_SLAVE_1 = 11;
        public static final int MOTOR_RIGHT_MASTER = 20;
        public static final int MOTOR_RIGHT_SLAVE_2 = 21;
    
        // suggested from tutorial
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.70;
    
        // from robot characteristics
        public static final double kS = 0.35; // static gain
        public static final double kV = 0.0055; // velocity gain
        public static final double kA = 0.000706; // acceleration gain
    
        public static final double kP = 0.0121; // 
        public static final double kD = 0.00572;
        public static final double MAX_VOLTAGE = 6.0;
        public static final double TRACK_WIDTH = 7.038154525;
    
        // verify these
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    }
    
}
