// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX shooterSpinMotor = new TalonFX(1);
    TalonFX indexerSpinMotor = new TalonFX(2);


    PositionVoltage shooterPosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public shooterSubsystem() {
    this.configureshooterSpinMotor(shooterSpinMotor);
    this.configureindexerSpinMotor(indexerSpinMotor);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void setShooterSpinSpeed(double shooterSpeed) {
    shooterSpinMotor.setControl(voltageRequest.withVelocity(shooterSpeed));
  }

   public double getShooterSpeed(){
    return shooterSpinMotor.getVelocity().getValueAsDouble();
  }

  public void setIndexerSpinSpeed(double indexerSpeed){
    indexerSpinMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
  }

  public double getIndexerSpeed(){
    return indexerSpinMotor.getVelocity().getValueAsDouble();
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
//######################################### INDEXER SPIN CONFIGURATION ###################################################### 

  public void configureindexerSpinMotor(TalonFX indexerMotor){
    TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();

    indexerMotorConfig.MotorOutput.Inverted = Constants.kIndexerMotorDirection;
    indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kIndexerMotorSupplyCurrentLimit;
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kIndexerMotorClosedLoopRampPeriod;
    indexerMotorConfig.Voltage.PeakForwardVoltage = Constants.kIndexerMotorPeakForwardVoltage;
    indexerMotorConfig.Voltage.PeakReverseVoltage = Constants.kIndexerMotorPeakReverseVoltage;
    

    Slot0Configs slot0 = indexerMotorConfig.Slot0;
    slot0.kP = Constants.kIndexerMotorProportional;
    slot0.kI = Constants.kIndexerMotorIntegral;
    slot0.kD = Constants.kIndexerMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kIndexerMotorVelocityFeedForward;
    slot0.kG = Constants.kIndexerMotorGravityFeedForward;
    slot0.kS = Constants.kIndexerMotorStaticFeedForward;
 


    StatusCode indexerMotorStatus = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      indexerMotorStatus = indexerMotor.getConfigurator().apply(indexerMotorConfig);
      if (indexerMotorStatus.isOK()) break;
    }
    if (!indexerMotorStatus.isOK()) {
      System.out.println("Could not configure device. Error: " + indexerMotorStatus.toString());
    }
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
