// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.commands.*;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final Shooter shooter = new Shooter();

  private final Limelight vision = new Limelight();
  
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;  
  
  public RobotContainer() {
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  private void configureBindings() {

        drivetrain.setDefaultCommand(
          new SwerveDriveControl(
            drivetrain, 
            () -> -joystick.getLeftX(),  //Translation 
            () -> -joystick.getLeftY(),  //Translation
            () -> -joystick.getRightX(), //Rotation
            joystick.povUp(), 
            joystick.povDown(), 
            joystick.y(), //Face Forward
            joystick.b(), //Face Right
            joystick.a(), //Face Backwards
            joystick.x()  //Face Left
          )
        );

        shooter.setDefaultCommand(new FeederControl(shooter, joystick.leftBumper(), joystick.leftTrigger()));


    joystick.rightTrigger().whileTrue(new ShooterControl(shooter, joystick.rightBumper()));
    joystick.rightTrigger().whileFalse(new InstantCommand(() -> shooter.ShooterStop()));
    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
     drivetrain.registerTelemetry(logger::telemeterize);
  }
  
  

  

  

  

  public Command getAutonomousCommand() {
      return new PathPlannerAuto("Optimized Test");
   }


}
