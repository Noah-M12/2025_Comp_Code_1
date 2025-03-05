
package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Run_ShooterSmall extends Command {

    private final double sw_speed;
    private final Shooter Shooter;

    public Run_ShooterSmall(Shooter shooter, double sw_speed) {
        this.Shooter = shooter;
        this.sw_speed = sw_speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
  
    }

    @Override
    public void execute() {
        Shooter.Small_Wheel_Shooter.set(-sw_speed);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.Small_Wheel_Shooter.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
