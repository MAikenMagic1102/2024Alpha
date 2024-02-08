// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private TalonFX m_ArmLeftMotor;
  private TalonFX m_ArmRightMotor;

  public Arm() {
    
    m_ArmLeftMotor = new TalonFX(Constants.Arm.leftArmMotor);
    m_ArmRightMotor = new TalonFX(Constants.Arm.rightArmMotor);

    m_ArmLeftMotor.setNeutralMode(null);
    












  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
