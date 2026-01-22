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

public class indexerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    
    TalonFX indexerSpinMotor = new TalonFX(0);

    PositionVoltage shooterPosition = new PositionVoltage(0);

    private VelocityVoltage voltageRequest = new VelocityVoltage(0);

  public indexerSubsystem() {
    this.configureindexerSpinMotor(indexerSpinMotor);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
 

  public void setIndexerSpinMotor(double indexerSpeed){
    indexerSpinMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
  }


  /*public void setTurretMotor(double indexerSpeed){
    indexerSpinMotor.setControl(voltageRequest.withVelocity(indexerSpeed));
  } this wont be used bc the turret will auto adjust with april tags */

//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ######################################################
//######################################### Start OF SHOOTER CONFIGURATION ###################################################### 

  //######################################### INDEXER SPIN CONFIGURATION ###################################################### 

  public void configureindexerSpinMotor(TalonFX indexerMotor){
    TalonFXConfiguration indexerMotorConfig = new TalonFXConfiguration();

    indexerMotorConfig.MotorOutput.Inverted = Constants.kindexerMotorDirection;
    indexerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kindexerMotorSupplyCurrentLimit;
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kindexerMotorClosedLoopRampPeriod;
    indexerMotorConfig.Voltage.PeakForwardVoltage = Constants.kindexerMotorPeakForwardVoltage;
    indexerMotorConfig.Voltage.PeakReverseVoltage = Constants.kindexerMotorPeakReverseVoltage;
    

    Slot0Configs slot0 = indexerMotorConfig.Slot0;
    slot0.kP = Constants.kindexerMotorProportional;
    slot0.kI = Constants.kindexerMotorIntegral;
    slot0.kD = Constants.kindexerMotorDerivative;

    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kV = Constants.kindexerMotorVelocityFeedForward;
    slot0.kG = Constants.kindexerMotorGravityFeedForward;
    slot0.kS = Constants.kindexerMotorStaticFeedForward;
 


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
