// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledMode = table.getEntry("ledMode");


  
  /** Creates a new Vision. */
  public Vision() {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
  

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  

  }
  // calculate height

  double distance;
  double ringHeight = 104.0; //104
  double visionHeight = 34.0; //34
  double fixedVisionAngle = 30;
  double differenceAngleY = ty.getDouble(0.0);
  double totalAngleY = fixedVisionAngle+differenceAngleY;
  boolean inRange;
  

  public void calculateDistance() { // finds the distance based off of the fixed angle and heights
    differenceAngleY = ty.getDouble(0.0);
    totalAngleY = fixedVisionAngle+differenceAngleY;
    distance =  (ringHeight-visionHeight) / Math.atan((totalAngleY*Math.PI)/180);
    if (Constants.minumumRange<distance && distance<Constants.maximumRange) {
      inRange = true;
    } else {
      inRange = false;
    }
    System.out.println(distance);
  }

  public double getDistance(){
    double gottenDistance;
    gottenDistance = distance;
    return gottenDistance;
  }
     
  public void turnLEDOff(){ // turns the green LEDs off
    ledMode.setNumber(1);
  }

  public void turnLEDOn(){ // turns the green LEDs on
    ledMode.setNumber(3);
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run//read values periodically
    double x = tx.getDouble(0.0); // gets the x angle, y angle, and target area
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    calculateDistance();

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putBoolean("Ramge", inRange);
    
  }
}
