package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;



/**
 * The 2017 turret shooter hood subsystem
 * @author Peter Dentch
 */
public class hoodServo extends SubsystemBase {

    TalonFX hoodMotor1 = new TalonFX(23, Constants.kCanivore);
    TalonFX hoodMotor2 = new TalonFX(24, Constants.kCanivore);

    private PositionVoltage hoodPos = new PositionVoltage(0);
 
    
    //SERVO Parameters from https://s3.amazonaws.com/actuonix/Actuonix+L16+Datasheet.pdf
    
    // pwm values in ms for the max and min angles of the shooter hood
    
    /**
     * Default constructor for the subsystem 
     */
    public hoodServo(){
    this.configurehoodMotor(hoodMotor1);
    this.configurehoodMotor2(hoodMotor2);
		
    }
	
    /**
     * Returns the shooter hood singleton object
     * @return is the current shooter hood object
     */
	
    /**
     * Takes a given angle and rotates the servo motor to that angle
     * @param degrees the angle limited by the min and max values defined in RobotMap
     */
    public void setPos1(double degrees){
	hoodMotor1.setControl(hoodPos.withPosition(degrees));
	hoodMotor2.setControl(hoodPos.withPosition(-degrees));
    }

     public double getPos(){
	return hoodMotor1.getPosition().getValueAsDouble();
    
	}
	

public void configurehoodMotor2(TalonFX hoodMotor2){
    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();

    hoodMotorConfig.MotorOutput.Inverted = Constants.kHoodMotorDirection;
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kHoodMotorSupplyCurrentLimit;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kHoodMotorVoltageClosedLoopRampPeriod;
    hoodMotorConfig.Voltage.PeakForwardVoltage = Constants.kHoodMotorMaxForwardVoltage;
    hoodMotorConfig.Voltage.PeakReverseVoltage = Constants.kHoodMotorMaxReverseVoltage;
    

    Slot0Configs slot0 = hoodMotorConfig.Slot0;
    slot0.kP = Constants.kHoodMotorProportional;
    slot0.kI = Constants.kHoodMotorIntegral;
    slot0.kD = Constants.kHoodMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kHoodMotorVelocityFeedForward;
    slot0.kS = Constants.kHoodMotorStaticFeedForward;
    slot0.kG = Constants.kHoodMotorGravityFeedForward;
 


    StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      hoodMotorStatus = hoodMotor2.getConfigurator().apply(hoodMotorConfig);
      if (hoodMotorStatus.isOK()) break;
    }
    if (!hoodMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + hoodMotorStatus.toString());
    }
    hoodMotor2.setPosition(0);
  }

     public void configurehoodMotor(TalonFX hoodMotor1){
    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();

    hoodMotorConfig.MotorOutput.Inverted = Constants.kHoodMotorDirection;
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kHoodMotorSupplyCurrentLimit;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kHoodMotorVoltageClosedLoopRampPeriod;
    hoodMotorConfig.Voltage.PeakForwardVoltage = Constants.kHoodMotorMaxForwardVoltage;
    hoodMotorConfig.Voltage.PeakReverseVoltage = Constants.kHoodMotorMaxReverseVoltage;
    

    Slot0Configs slot0 = hoodMotorConfig.Slot0;
    slot0.kP = Constants.kHoodMotorProportional;
    slot0.kI = Constants.kHoodMotorIntegral;
    slot0.kD = Constants.kHoodMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kHoodMotorVelocityFeedForward;
    slot0.kS = Constants.kHoodMotorStaticFeedForward;
    slot0.kG = Constants.kHoodMotorGravityFeedForward;
 


    StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      hoodMotorStatus = hoodMotor1.getConfigurator().apply(hoodMotorConfig);
      if (hoodMotorStatus.isOK()) break;
    }
    if (!hoodMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + hoodMotorStatus.toString());
    }
    hoodMotor1.setPosition(0);
  }
    /**
     * Returns the current angle of the servo by taking the angle it was last set to
     * with the time before the movement begins and after that is called the current
     * time and angle the servo is moving to is taken and the current angle is estimated
     * @return the estimated current angle of the servo in degrees
     */
   
    /**
     * Sets the default command of the subsystem
     */
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new DriveHoodWithJoystick());
    }
}
