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

    public static final class ShooterConstants {
        public static final int shooter1Port = 30;
        public static final int shooter2Port = 31;
    }

    public static final class IntakeConstants {
        public static final int intakePort = 50;
    }

    public static final class HopperConstants {
        public static final int hopperPort = 51;
        public static final double hopperDefaultPercentOutput = 0.8;
    }

    public static final class FeederConstants {
        public static final int feederPort = 40;
    }

    public static final class OIConstants {
        public static final int xboxControllerPort = 0;
  
        public enum Button {
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
        } 
    }
}
