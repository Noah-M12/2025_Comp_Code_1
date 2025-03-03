
package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Run_Shooter extends Command {

    private final double bw_speed;
    private final double sw_speed;
    private final Shooter Shooter;

    public Run_Shooter(Shooter shooter, double bw_speed, double sw_speed) {
        this.Shooter = shooter;
        this.bw_speed = bw_speed;
        this.sw_speed = sw_speed;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
  
    }

    @Override
    public void execute() {

        Shooter.Big_Wheel_Shooter.set(-bw_speed);
        Shooter.Small_Wheel_Shooter.set(-sw_speed);
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.Big_Wheel_Shooter.set(0);
        Shooter.Small_Wheel_Shooter.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
