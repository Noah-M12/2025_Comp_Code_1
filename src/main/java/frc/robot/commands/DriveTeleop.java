package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveTeleop extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_drive;
  private final Supplier<Double> m_xSpeedFunction, m_ySpeedFunction, m_rotFunction;
  //private final Supplier<Boolean> m_fieldRelativeFunction, m_rateLimitFunction;
  private final boolean m_fieldRelative;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  //Supplier<Double> ySpeedFunction, Supplier<Double> rotFunction, Supplier<Boolean> fieldRelativeFunction, Supplier<Boolean> rateLimitFunction
  public DriveTeleop(
      DriveSubsystem driveSubsystem, 
      Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> rotFunction,
      boolean fieldRelativeFunction
      
      ) {
    m_drive = driveSubsystem;
    m_xSpeedFunction = xSpeedFunction;
    m_ySpeedFunction = ySpeedFunction;
    m_rotFunction = rotFunction;
    m_fieldRelative = fieldRelativeFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drive.drive(m_xSpeedFunction.get(), 
    m_ySpeedFunction.get(), m_rotFunction.get(), 
    m_fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
