// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public static Spark m_RightLED;
  public static Spark m_LeftLED;
  public static double LEDPattern; //See 5 LED PATTERN TABLE: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  public static double LEDBlink;

  static double lastTime = -1;

  public LEDs() {
    // Must be a PWM header, not MXP or DIO
    m_RightLED = new Spark(0);
    m_LeftLED = new Spark(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Set the LEDs
    //m_RightLED.set(LEDPattern);
    //m_LeftLED.set(LEDPattern);
  }

  public static Command setLED(double LEDPattern) {
      m_RightLED.set(LEDPattern);
      m_LeftLED.set(LEDPattern);
      return null;
    }

  public static Command setLEDwBlink(double LEDPattern, double LEDBlink) {
      if (lastTime != -1 && Timer.getFPGATimestamp() - lastTime <= LEDBlink) {
        m_RightLED.set(LEDPattern);
        m_LeftLED.set(LEDPattern);
      }
      else if (Timer.getFPGATimestamp() - lastTime <= LEDBlink * 2) {
        m_RightLED.set(.99);
        m_LeftLED.set(.99);
      }      
      else {
        lastTime = Timer.getFPGATimestamp();
      }
      return null;
    };
}