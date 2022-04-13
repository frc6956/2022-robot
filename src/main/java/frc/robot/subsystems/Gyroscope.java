// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*
public class WPI_PigeonIMU extends PigeonIMU implements Sendable, Gyro{
  // Creates a new Gyroscope. 

  private PigeonIMU pigeon = new PigeonIMU(0);
	double zeroAngle;

  public WPI_PigeonIMU() {
    reset();
  }

    
  public void displayAngle() {
    SmartDashboard.putNumber("Angle:", getAngle());
  }


	public void calibrate() {
	}
	
	 // sets current angle to zero
	 
	
	public void reset() {
		zeroAngle = getRawAngle();
	}
	
	 // calculates current angle
	 // returns a double value of the current angle
	 //

	public double getAngle() {
		return getRawAngle() - zeroAngle;
	}
	
	 //gets the rate of the robot
	 //returns double value of the robot rate
	 

	public double getRate() {
		return 0;
	}

	
	 // calculates total rotation of robot
	// @return double total angle
	 
	private double getRawAngle() {
		double [] ypr = new double [3];
		pigeon.getYawPitchRoll(ypr);
		return -ypr[0];
	}


	public void close() throws Exception {

	}




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    displayAngle();
  }
  
}
*/