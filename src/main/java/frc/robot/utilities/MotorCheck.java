/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Add your docs here.
 */
public class MotorCheck {
    private WPI_TalonFX motor1;
    private WPI_TalonFX motor2;
    private WPI_VictorSPX victorMotor;
    private int victorMotorPDP;
    private int motor2PDP;
    private String subsystemChecked;

    public MotorCheck(WPI_TalonFX motor1, WPI_TalonFX motor, String subsystem) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.subsystemChecked = subsystem;
    }

    public MotorCheck(WPI_VictorSPX victorMotor, int victorMotorPDP, String subsystem) {
        this.victorMotor = victorMotor;
        this. victorMotorPDP = victorMotorPDP;
        this.subsystemChecked = subsystem;
    }

    public void checkMotors() {
        /**
   * Checks drive motor currents, records sticky faults if a motor is faulty for more than 5 cycles
   * @param motor1PDP RobotMap PDP address for motor1
   * @param motor2PDP RobotMap PDP address for motor2
   * @param motor3PDP RobotMap PDP address for motor3
   * @param side true is left, false is right
   */
	/* public void verifyMotors(int motor1PDP, int motor2PDP, WPI_TalonFX motor1, WPI_TalonFX motor2; boolean side) {
        double amps1 = pdp.getCurrent(motor1PDP);
        double amps2 = pdp.getCurrent(motor2PDP);
        double averageAmps = (amps1 + amps2) / 2;
    
        double temp1 = motor1.getTemperature();
        double temp2 = motor1.getTemperature();
        double averageTemp = (temp1 + temp2) / 2;
    
        if(leftMotorFaultCount >= 5) {
          Robot.robotPrefs.recordStickyFaults("Left" + " Drive Train");
          leftMotorFaultCount = 0;
        } else if (rightMotorFaultCount >= 5) {
          Robot.robotPrefs.recordStickyFaults("Right" + " Drive Train");
          rightMotorFaultCount = 0;
        }
        if(averageAmps > 7) {
          if(amps1 < 4 || amps2 < 4 || amps3 < 4) {
            if(side) leftMotorFaultCount++;
            else  rightMotorFaultCount++;
          }
          else {
            if (side) leftMotorFaultCount = 0;
            else rightMotorFaultCount = 0;
          }
      }
    }*/
    }

}
