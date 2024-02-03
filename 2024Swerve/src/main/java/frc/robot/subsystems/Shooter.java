// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private TalonFX Shooter1, Shooter2;
  private TalonFX Feeder;
  private DigitalInput noteSensor;
  private PhoenixPIDController shooterPID;

  private VelocityVoltage shooterSpeed;
  
  /** Creates a new Shooter. */
  public Shooter(){

    Shooter1 = new TalonFX(Constants.Shooter.Shooter1_ID);
    Shooter2 = new TalonFX(Constants.Shooter.Shooter2_ID);
    Feeder = new TalonFX(Constants.Shooter.Feeder_ID);
    noteSensor = new DigitalInput(Constants.Shooter.noteSensor_DIO);
    shooterPID = new PhoenixPIDController(Constants.Shooter.Shooter_kP, Constants.Shooter.Shooter_kI, Constants.Shooter.Shooter_kD);
    Shooter1.setInverted(Constants.Shooter.Shooter1_Inverted);
    Shooter2.setInverted(Constants.Shooter.Shooter2_Inverted);
    Feeder.setInverted(true);
    Shooter2.setControl(new Follower(Constants.Shooter.Shooter1_ID, true));
    shooterSpeed = new VelocityVoltage(0);
  }

  public void feederIn(){
    Feeder.set(0.4);
  }

  public void feederOut(){
    Feeder.set(-0.4);
  }

  public void feederStop(){
    Feeder.set(0.0);
  }

  public void feederShoot(boolean ready){
    if(ready){
      Feeder.set(0.8);
    }
    else{
      Feeder.set(0.0);
    }
  }

  public void ShooterStop(){
    Shooter1.set(0.0);
  }

  public void setShooterThrottle(double throttle){
    Shooter1.set(throttle);
  }

  public void setShooterSpeed(double speed){
    Shooter1.setControl(shooterSpeed.withVelocity(speed));
  } 
  
  public void setShooterDistance(double distance){
    
  }

  public boolean getShooterReady(){
    return Math.abs(Shooter1.getClosedLoopError().getValueAsDouble()) < 100;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
