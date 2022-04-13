// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.*;

/** Add your docs here. */
public class SendableCANSparkMax extends CANSparkMax implements Sendable {
  int deviceID;

  public SendableCANSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    this.deviceID = deviceId;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(this::disable);
    builder.addDoubleProperty("Value", this::get, this::set);
    
    builder.addBooleanProperty("Inverted", () -> this.getInverted(), (value) -> this.setInverted(value));
    builder.addDoubleProperty("BusVoltage", () -> this.getBusVoltage(), null);
    builder.addDoubleProperty("Output", () -> this.getAppliedOutput(), null);
    builder.addDoubleProperty("OutputVoltage", () -> this.getAppliedOutput() * this.getBusVoltage(), null);
    builder.addDoubleProperty("OutputCurrent", () -> this.getOutputCurrent(), null);
    builder.addDoubleProperty("Temperature", () -> this.getMotorTemperature(), null);

    RelativeEncoder encoder = this.getEncoder();
    builder.addDoubleProperty("Velocity", () -> encoder.getVelocity(), null);
    builder.addDoubleProperty("Position", () -> encoder.getPosition(), (value) -> encoder.setPosition(value));

    SparkMaxPIDController pid = this.getPIDController();
    builder.addDoubleProperty("P", () -> pid.getP(), (value) -> pid.setP(value));
    builder.addDoubleProperty("I", () -> pid.getI(), (value) -> pid.setI(value));
    builder.addDoubleProperty("D", () -> pid.getD(), (value) -> pid.setD(value));
    builder.addDoubleProperty("FF", () -> pid.getFF(), (value) -> pid.setFF(value));
    builder.addDoubleProperty("OutputMin", () -> pid.getOutputMin(), (value) -> pid.setOutputRange(value, pid.getOutputMax()));
    builder.addDoubleProperty("OutputMax", () -> pid.getOutputMax(), (value) -> pid.setOutputRange(pid.getOutputMin(), value));
    builder.addDoubleProperty("IMaxAccum", () -> pid.getIMaxAccum(0), (value) -> pid.setIMaxAccum(value, 0));
    builder.addDoubleProperty("IZone", () -> pid.getIZone(), (value) -> pid.setIZone(value));
  }
}
