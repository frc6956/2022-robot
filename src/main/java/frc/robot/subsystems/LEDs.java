// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {

  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  double m_firstPixelHue1 = 0; // sets the beginning hue to red
  
  double m_yellowGreenFirstPixelHue = 0;



  /** Creates a new LEDs. */
  public LEDs() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output // 47
    m_ledBuffer = new AddressableLEDBuffer(47);


    setUpLight();
  }

  public void setUpLight() { // gets LED segment length and sets the data for them
    // Length is expensive to set, so only set it once, then just update data
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  public void setAllGreen(){ // sets all LEDs to green
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }

   public void setYellowToGreen(){ // runs LEDs that change colors from red to green 
      // For every pixel
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = ((int)(m_yellowGreenFirstPixelHue + (i * 55 / m_ledBuffer.getLength())) % 55) + 20; // hue is red to green
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      
      }
      // Increase by to make the rainbow "move"
      m_yellowGreenFirstPixelHue += 2;
      // Check bounds
      m_yellowGreenFirstPixelHue %= 55;
  }


  public void shooterColorSpeed(double rpm){ // runs LEDs in which the shooter rpm controls how fast it changes color
    int shooterMaxRPM = 6000;
    int shooterMinRPM = 0;
    double hueSpeed;
    int hueSpeedMin = 1;
    int hueSpeedMax = 6;
  
    //normalize the rpm and convert a range of 0-6000 to a range of 1-6
  hueSpeed = ((rpm - shooterMinRPM)/(shooterMaxRPM - shooterMinRPM))*(hueSpeedMax - hueSpeedMin) + hueSpeedMin;


    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (m_firstPixelHue1 > 71){
        if (rpm > 3000){
          m_firstPixelHue1 = 55;
        } else {
          m_firstPixelHue1 = 0;
        }
      }
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (int)(m_firstPixelHue1 + (i * 65 / m_ledBuffer.getLength())) % 65; // hue is red to green
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);

    }
    // Increase by to make the rainbow "move"
    m_firstPixelHue1 += hueSpeed; //the hue changes color slower or quicker depending on the rpm of the shooter
    // Check bounds
    m_firstPixelHue1 %= 71;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  
  }
}
