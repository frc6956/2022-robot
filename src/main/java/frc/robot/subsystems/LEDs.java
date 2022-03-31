// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDs extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  double m_firstPixelHue1 = 15; // sets the beginning hue to red
  
  double m_yellowGreenFirstPixelHue = 0;

  double m_rainbowFirstPixelHue = 0;

  double m_autonRPulseBlue = 0;

  double m_autonRPulseRed = 0;



  /** Creates a new LEDs. */
  public LEDs() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(1);
    


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
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, 255, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }

  public void setAllGold(){ // sets all LEDs to gold
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for gold
      m_ledBuffer.setRGB(i, 255, 215, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }


  public void setAllOff(){ // sets all LEDs to green
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 0, 0, 0);
   }
   
   m_led.setData(m_ledBuffer);
  }

  public void setRainbowColors(){ // sets all LEDs to pride colors
    for (var i = 0; i < (m_ledBuffer.getLength())/6; i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 209, 34, 41);
    }

   for (var i = (m_ledBuffer.getLength())/6; i < (m_ledBuffer.getLength())/6*2; i++) {
      // Sets the specified LED to the RGB values for orange
      m_ledBuffer.setRGB(i, 246, 138, 30);
    }

   for (var i = (m_ledBuffer.getLength())/6*2; i < (m_ledBuffer.getLength())/6*3; i++) {
      // Sets the specified LED to the RGB values for yellow
      m_ledBuffer.setRGB(i, 253, 224, 26);
    }

   for (var i = (m_ledBuffer.getLength())/6*3; i < (m_ledBuffer.getLength())/6*4; i++) {
      // Sets the specified LED to the RGB values for green
      m_ledBuffer.setRGB(i, 0, 121, 64);
    }

   for (var i = (m_ledBuffer.getLength())/6*4; i < (m_ledBuffer.getLength())/6*5; i++) {
      // Sets the specified LED to the RGB values for blue
      m_ledBuffer.setRGB(i, 36, 64, 142);
    }

   for (var i = (m_ledBuffer.getLength())/6*5; i < (m_ledBuffer.getLength())/6*6; i++) {
      // Sets the specified LED to the RGB values for purple
      m_ledBuffer.setRGB(i, 115, 41, 130);
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

  public void autonPulseRed(){ // runs LEDs that change colors from red to green 
    /*
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  
      // shape is a circle so only one value needs to precess
      final int saturation = ((int)(m_autonRPulseRed + (i * 45 / m_ledBuffer.getLength())) % 45) + 100; // hue is red to green
      // Set the value
      m_ledBuffer.setHSV(i, 0, saturation, 64);
    
    }
    // Increase by to make the rainbow "move"
    m_autonRPulseRed += 2;
    // Check bounds
    m_autonRPulseRed %= 45;
    */
    
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for gold
        m_ledBuffer.setRGB(i, 205, 0, 0);
      }
  }

  public void autonPulseBlue(){ // runs LEDs that change colors from red to green 
    /*
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
  
      // shape is a circle so only one value needs to precess
      final int saturation = ((int)(m_autonRPulseBlue + (i * 45 / m_ledBuffer.getLength())) % 45) + 100; // hue is red to green
      // Set the value
      m_ledBuffer.setHSV(i, 75, saturation, 64);
    
    }
    // Increase by to make the rainbow "move"
    m_autonRPulseBlue += 2;
    // Check bounds
    m_autonRPulseBlue %= 45;
    */
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for gold
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
}


  public void rainbow(){ // runs LEDs that change colors from red to green 
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180; // hue is red to green
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
}

public void superRainbow(){ // runs LEDs that change colors from red to green 
  // For every pixel
  for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // Calculate the hue - hue is easier for rainbows because the color
    // shape is a circle so only one value needs to precess
    final int hue = ((int)(m_rainbowFirstPixelHue) + (i * 180 / m_ledBuffer.getLength())) % 180; // hue is red to green
    // Set the value
    m_ledBuffer.setHSV(i, hue, 255, 128);
  
  }
  // Increase by to make the rainbow "move"
  m_rainbowFirstPixelHue += 9;
  // Check bounds
  m_rainbowFirstPixelHue %= 180;
}


  public void shooterColorSpeed(double rpm){ // runs LEDs in which the shooter rpm controls how fast it changes color
    double shooterMaxRPM = (Constants.ShooterMotorRightSpeed * 6000);
    int shooterMinRPM = 0;
    double hueSpeed;
    int hueSpeedMin = 1;
    int hueSpeedMax = 9;
  
    //normalize the rpm and convert a range of 0-6000 to a range of 1-6
  hueSpeed = ((rpm - shooterMinRPM)/(shooterMaxRPM - shooterMinRPM))*(hueSpeedMax - hueSpeedMin) + hueSpeedMin;

    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if (m_firstPixelHue1 > 71){
        if (rpm > 3000){
          m_firstPixelHue1 = 55;
        } else {
          m_firstPixelHue1 = 15;
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
