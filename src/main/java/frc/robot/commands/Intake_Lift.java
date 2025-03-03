package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneunamatics;

public class Intake_Lift extends Command{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final Pneunamatics pneunamatics; 
    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intake_Lift(Pneunamatics pneunamatics) {
    this.pneunamatics = pneunamatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pneunamatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneunamatics.togglesolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; 
  }

}
