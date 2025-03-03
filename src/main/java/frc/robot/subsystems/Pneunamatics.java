package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class Pneunamatics extends SubsystemBase {

    private Compressor compressor;
    private final DoubleSolenoid intake_lift;  
    private PneumaticHub ph; 
    private double x; 
    

    public Pneunamatics(){ 
        ph = new PneumaticHub(2); 
        //compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor = ph.makeCompressor();
        //intake_lift = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6,7);
        intake_lift = ph.makeDoubleSolenoid(6,7);


        ph.enableCompressorAnalog(100, 118);
        intake_lift.set(DoubleSolenoid.Value.kForward); // this is setting the piston to down 

    }
    public void initIntakeSolenoid() {
        intake_lift.set(DoubleSolenoid.Value.kReverse);
    }

    public void enableCompressor(){
        ph.enableCompressorAnalog(100, 118);
    }

    public void disableCompressor(){
        ph.disableCompressor();
    }
    public void togglesolenoid(){
        intake_lift.toggle();
        
    }

    public boolean compressorEnabled() {
        return compressor.isEnabled();
    }

    public double getPressure(){
        x = compressor.getPressure();
        return x; 
    }

    public void printData(SmartDashboard s) {
        s.putNumber("Pressure", getPressure());
    }

    
}
