// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class shooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX shooterSpinMotor = new TalonFX(0);
    TalonFX hoodMotor = new TalonFX(1);
    TalonFX turretMotor = new TalonFX(1);

    PositionVoltage shooterPosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public shooterSubsystem() {
    this.configureshooterSpinMotor(shooterSpinMotor);
    this.configurehoodMotor(hoodMotor);
    this.configureturretMotor(turretMotor);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setShooterSpinSpeed(double shooterSpeed) {
    shooterSpinMotor.setControl(voltageRequest.withVelocity(shooterSpeed));
  }

  public void setHoodMotor(double hoodPos){
    hoodMotor.setControl(shooterPosition.withVelocity(hoodPos));
  }

  public void setTurretMotorPos(double indexerSpeed){
    turretMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
  } 

  public double getHoodMotorPos(){
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public double getTurretMotorPos(){
    return turretMotor.getPosition().getValueAsDouble();
  }

//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ###################################################### 

//######################################### SHOOTER SPIN CONFIGURATION ###################################################### 


  public void configureshooterSpinMotor(TalonFX shooterSpinMotor){
    TalonFXConfiguration shooterSpinMotorConfig = new TalonFXConfiguration();

    shooterSpinMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kShooterSpinMotorSupplyCurrentLimit;
    shooterSpinMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    shooterSpinMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kShooterSpinMotorClosedLoopRampPeriod;
    shooterSpinMotorConfig.Voltage.PeakForwardVoltage = Constants.kShooterSpinMotorPeakForwardVoltage;
    shooterSpinMotorConfig.Voltage.PeakReverseVoltage = Constants.kShooterSpinMotorPeakReverseVoltage;

    shooterSpinMotorConfig.MotorOutput.Inverted = Constants.kShooterSpinMotorDirection;
    shooterSpinMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = shooterSpinMotorConfig.Slot0;
    slot0.kP = Constants.kShooterSpinMotorProportional;
    slot0.kI = Constants.kShooterSpinMotorIntegral;
    slot0.kD = Constants.kShooterSpinMotorDerivative;
    slot0.kV = Constants.kShooterSpinMotorVelocityFeedForward;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kShooterSpinMotorGravityFeedForward;
    slot0.kS = Constants.kShooterSpinMotorStaticFeedForward;
    

    
    StatusCode shooterSpinMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      shooterSpinMotorStatus = shooterSpinMotor.getConfigurator().apply(shooterSpinMotorConfig);
      if (shooterSpinMotorStatus.isOK()) break;
    }
    if (!shooterSpinMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + shooterSpinMotorStatus.toString());
    }
  }

  //######################################### HOOD SPIN CONFIGURATION ###################################################### 

  public void configurehoodMotor(TalonFX hoodMotor){
    TalonFXConfiguration hoodMotorConfig = new TalonFXConfiguration();

    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kHoodMotorSupplyCurrentLimit;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    hoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kHoodMotorClosedLoopRampPeriod;
    hoodMotorConfig.Voltage.PeakForwardVoltage = Constants.kHoodMotorPeakForwardVoltage;
    hoodMotorConfig.Voltage.PeakReverseVoltage = Constants.kHoodMotorPeakReverseVoltage;

    hoodMotorConfig.MotorOutput.Inverted = Constants.kHoodMotorDirection;
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = hoodMotorConfig.Slot0;
    slot0.kP = Constants.kHoodMotorProportional;
    slot0.kI = Constants.kHoodMotorIntegral;
    slot0.kD = Constants.kHoodMotorDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kHoodMotorGravityFeedForward;
    slot0.kV = Constants.kHoodMotorVelocityFeedForward;
    slot0.kS = Constants.kHoodMotorStaticFeedForward;
    

    
    StatusCode hoodMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      hoodMotorStatus = hoodMotor.getConfigurator().apply(hoodMotorConfig);
      if (hoodMotorStatus.isOK()) break;
    }
    if (!hoodMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + hoodMotorStatus.toString());
    }
    hoodMotor.setPosition(0);
  }

  public void configureturretMotor(TalonFX turretMotor){
    TalonFXConfiguration turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kTurretMotorSupplyCurrentLimit;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;    

    turretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kTurretMotorClosedLoopRampPeriod;
    turretMotorConfig.Voltage.PeakForwardVoltage = Constants.kTurretMotorPeakForwardVoltage;
    turretMotorConfig.Voltage.PeakReverseVoltage = Constants.kTurretMotorPeakReverseVoltage;

    turretMotorConfig.MotorOutput.Inverted = Constants.kTurretMotorDirection;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  

    Slot0Configs slot0 = turretMotorConfig.Slot0;
    slot0.kP = Constants.kTurretMotorProportional;
    slot0.kI = Constants.kTurretMotorIntegral;
    slot0.kD = Constants.kTurretMotorDerivative;
    
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kG = Constants.kTurretMotorGravityFeedForward;
    slot0.kV = Constants.kTurretMotorVelocityFeedForward;
    slot0.kS = Constants.kTurretMotorStaticFeedForward;
    

    
    StatusCode turretMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      turretMotorStatus = turretMotor.getConfigurator().apply(turretMotorConfig);
      if (turretMotorStatus.isOK()) break;
    }
    if (!turretMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + turretMotorStatus.toString());
    }
    turretMotor.setPosition(0);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
