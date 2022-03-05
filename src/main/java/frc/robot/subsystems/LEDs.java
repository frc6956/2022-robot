// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {

  private AddressableLED m_led;
 // private AddressableLED m_led2;
 // private AddressableLED m_led3;
  private AddressableLEDBuffer m_ledBuffer;


  /** Creates a new LEDs. */
  public LEDs() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);
   // m_led2 = new AddressableLED(8);
   // m_led3 = new AddressableLED(7);
    // Reuse buffer
    // Default to a length of 60, start empty output // 47
    m_ledBuffer = new AddressableLEDBuffer(47);

    setUpLight();
  }

  public void setUpLight() { // gets LED segment length and sets the data for them
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength());
   // m_led2.setLength(m_ledBuffer.getLength());
   // m_led3.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

   // m_led2.setData(m_ledBuffer);
   // m_led2.start();

   // m_led3.setData(m_ledBuffer);
  //  m_led3.start();
  }

  public void setAllGreen(){ // sets all LEDs to green
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 255);
   }
   
   m_led.setData(m_ledBuffer);
   //m_led2.setData(m_ledBuffer);
   //m_led3.setData(m_ledBuffer);
  }

  
  public void setRedToGreen(){ // runs LEDs that change colors from red to green 
    double m_rainbowFirstPixelHue = 0;
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (int)(m_rainbowFirstPixelHue + (i * 60 / m_ledBuffer.getLength())) % 60; // hue is red to green
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 60;
  }

  public void shooterColorSpeed(double rpm){ // runs LEDs in which the shooter rpm controls how fast it changes color
    double m_firstPixelHue = 0; // sets the beginning hue to red
    int shooterMaxRPM = 6000;
    int shooterMinRPM = 0;
    double hueSpeed;
    int hueSpeedMin = 1;
    int hueSpeedMax = 6;
  
    //normalize the rpm and convert a range of 0-6000 to a range of 1-6
  hueSpeed = ((rpm - shooterMinRPM)/(shooterMaxRPM - shooterMinRPM))*(hueSpeedMax - hueSpeedMin) + hueSpeedMin;

    // changes starting hue based on current speed
    if (rpm < 1000){
      m_firstPixelHue = 0; // red
    }
    else if (rpm < 3000){
      m_firstPixelHue = 20; // red orange
    }
    else if (rpm < 5000){
      m_firstPixelHue = 25; // orange
    }
    else{
      m_firstPixelHue = 35; // yellow
    }
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (int)(m_firstPixelHue + (i * 60 / m_ledBuffer.getLength())) % 60; // hue is red to green
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_firstPixelHue += hueSpeed; //the hue changes color slower or quicker depending on the rpm of the shooter
    // Check bounds
    m_firstPixelHue %= 60;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setRedToGreen();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    //m_led2.setData(m_ledBuffer);
    //m_led3.setData(m_ledBuffer);
  }
}
