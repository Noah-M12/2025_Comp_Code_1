package frc.robot.commands;

import frc.robot.LimelightHelpers; 
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;


import edu.wpi.first.wpilibj2.command.Command;

public class LimeLightTargeting extends Command { 
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final DriveSubsystem m_drive;

    public double desired_distance; // The Distance from robot to April Tag
    private double targetOffsetAngle_Vertical; 
    private double limelight_Mount_Angle_Degrees = 15.0; 
    private double limelight_Lens_From_Ground = 13.375; 
    private double limelight_from_edge_of_bumper_offset = 6; 
    private double Apriltag_Height_Inches = 53; 
    private double angleToGoalDegrees; 
    private double angleToGoalRadians; 
    private double distance_From_Limelight_To_Goal_Inches; 
    private double targetingForwardSpeed; 
    private double distance_error; 
    private double distance_error_before_drivekp; 
    

    // PID Values 
    private double kP_Turning = .003; // Turning
    private double drive_kP = 0.01; 
    private double targetAngularVelocity; 

    public LimeLightTargeting( DriveSubsystem driveSubsystem, double desired_distance) {

        m_drive = driveSubsystem; 
        desired_distance = desired_distance; 
        //m_xSpeedFunction = xSpeedFunction;
        //m_ySpeedFunction = ySpeedFunction;
    
    
       // m_drive.setIntakeMode("VisionTurn");
    
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
      }

      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Switch to pipeline 0
  LimelightHelpers.setPipelineIndex("", 0);
    //SmartDashboard.putNumber("Desired Distance", desired_distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
    double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If target is on the rightmost edge of 
    // limelight 3A feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveSubsystem.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= DriveSubsystem.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
            DriveSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
         DriveSubsystem.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        
       DriveSubsystem.kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods

    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

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
