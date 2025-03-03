package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Pneunamatics;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PrintTelemetry extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Pneunamatics pneunamatics;
  //private final Supplier<Integer> presSupplier;
  //private final Supplier<Boolean> m_fieldRelativeFunction, m_rateLimitFunction;
  
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  //Supplier<Double> ySpeedFunction, Supplier<Double> rotFunction, Supplier<Boolean> fieldRelativeFunction, Supplier<Boolean> rateLimitFunction
  public PrintTelemetry(Pneunamatics pneunamatics) {
    this.pneunamatics = pneunamatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneunamatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Pressure",pneunamatics.getPressure());
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
