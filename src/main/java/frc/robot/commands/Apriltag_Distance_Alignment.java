package frc.robot.commands;

import frc.robot.LimelightHelpers; 
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.Command;

public class Apriltag_Distance_Alignment extends Command { 
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

    public Apriltag_Distance_Alignment( DriveSubsystem driveSubsystem, double desired_distance) {

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
  public void execute() {

    

    //Vision Math 

      //Rotation Math 

        // Get the Angle from the Limelight and multiple by Turning P value 
        targetAngularVelocity = LimelightHelpers.getTX("limelight-llthree") * kP_Turning; // Get the Angle from the Limelight and multiple by Turning P value 
        // Mulitply by Angular Speed 
        targetAngularVelocity *= Constants.DriveConstants.kMaxAngularSpeed; 
        //Inverse for correct direction and display to smart dashboard
        targetAngularVelocity *= -1.0; 
        SmartDashboard.putNumber("targetAngularVelocity", targetAngularVelocity); 

    //Distance Math 
        //Grab y angle from Limelight
        targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-llthree"); 
        // Add the mounting angle and the value from the limelight 
        angleToGoalDegrees = limelight_Mount_Angle_Degrees + targetOffsetAngle_Vertical; 
        // Convert to Radians because the Tan function only takes in radians 
        angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance 

    // Plug into  D =  (Known Field Height - Limelight from the Ground) / tan( Mounting Angle + Limelight Y Angle) 
    //  distance_From_Limelight_To_Goal_Inches = (Apriltag_Height_Inches - limelight_Lens_From_Ground) 
      //  / Math.tan(angleToGoalRadians); 

    //Distance Error 
    // the reaons why there is a offset for the limelight and the bumper is because we can get even more accrute 

    //  We all add the distance  solved for with  how far the Limelight is from the bumper and subtract from where we want to be 
    distance_error =  (distance_From_Limelight_To_Goal_Inches + limelight_from_edge_of_bumper_offset ) - desired_distance; 
    // Store our value before applying a proportional value to it 
    distance_error_before_drivekp = distance_error; 
    distance_error = distance_error * drive_kP; 

    //System.out.print(desired_distance);
    SmartDashboard.putNumber("Desired Distance", desired_distance);
    SmartDashboard.putNumber("Distance From Limelight", distance_From_Limelight_To_Goal_Inches);
    SmartDashboard.putNumber("distance_error_before_drivekp", distance_error_before_drivekp);
    SmartDashboard.putNumber("Distance_Error", distance_error);


      m_drive.drive(distance_error,  0, targetAngularVelocity, false);
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
