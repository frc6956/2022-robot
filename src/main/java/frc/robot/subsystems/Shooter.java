// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkMax ShooterMotorRight; // Creates new motor 
  private CANSparkMax ShooterMotorLeft;
  
  public Shooter() { // Defines both the Ids and the type of motors that were created above
     ShooterMotorLeft = new CANSparkMax(Constants.ShooterMotorLeftID, MotorType.kBrushless);
     ShooterMotorRight = new CANSparkMax(Constants.ShooterMotorRightID, MotorType.kBrushless);
     ShooterMotorLeft.follow(ShooterMotorRight, true);
  }
  
  public void shoot(){ // Sets speed of motors to the speed constants when called upon
    ShooterMotorRight.set(Constants.ShooterMotorRightSpeed);
  }

  public void stop(){ // Stops the motors when calle upon
    ShooterMotorLeft.set(0);
    ShooterMotorRight.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
