package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;


public class Constants {
    public static final double[] kBlueHubCoord = {4.627, 4.024};
    public static final double[] kRedHubCoord = {4.627, 4.024};
    public static final double[] kBluePassCoords = {1.905, 6.222, 2.059};


/*
 * ####################################################################################################################################
 * ##################################################### Positions ###################################################################
 * ####################################################################################################################################
*/

    public static final double kIntakeSlidePos = 0.5;
    public static final double kIndexerMainSpeed = 10;
    public static final double kIndexerNoSpeed = 0;




/*
 * ####################################################################################################################################
 * ####################################################### Intake Spin ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kIntakeSpinMotorProportional = 0;
    public static final double kIntakeSpinMotorIntegral = 0;
    public static final double kIntakeSpinMotorDerivative = 0;

    public static final double kIntakeSpinMotorVelocityFeedForward = 0.122;
    public static final double kIntakeSpinMotorStaticFeedForward = 0;
    public static final double kIntakeSpinMotorGravityFeedForward = 0;

    public static final double kIntakeSpinMotorPeakForwardVoltage = 11;
    public static final double kIntakeSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kIntakeSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIntakeSpinMotorSupplyCurrentLimit = 40;

    public static final double kIntakeSpinMotorClosedLoopRampPeriod = 0;
    
  
/*
 * ####################################################################################################################################
 * ####################################################### Intake Slide ###############################################################
 * ####################################################################################################################################
*/

    public static final double kIntakeElectricSlideMotorSupplyCurrentLimit = 40;
    public static final double kIntakeElectricSlideMotorVoltageClosedLoopRampPeriod = 0;
    public static final double kIntakeElectricSlideMotorMaxForwardVoltage = 11;
    public static final double kIntakeElectricSlideMotorMaxReverseVoltage = -11;
    public static final double kIntakeElectricSlideMotorVelocityFeedForward = 0;
    public static final double kIntakeElectricSlideMotorGravityFeedForward = 0;
    public static final double kIntakeElectricSlideMotorStaticFeedForward = 0;

    public static final InvertedValue kIntakeElectricSlideMotorDirection = InvertedValue.Clockwise_Positive;

    public static final double kIntakeElectricSlideMotorProportional = 0;
    public static final double kIntakeElectricSlideMotorIntegral = 0;
    public static final double kIntakeElectricSlideMotorDerivative = 0;
    
/*
 * ####################################################################################################################################
 * ####################################################### Shooter Spin ###############################################################
 * ####################################################################################################################################
*/

    public static final double kShooterSpinMotorSupplyCurrentLimit = 0;
    public static final double kShooterSpinMotorClosedLoopRampPeriod = 0;
    public static final double kShooterSpinMotorPeakForwardVoltage = 0;
    public static final double kShooterSpinMotorPeakReverseVoltage = 0;
    public static final InvertedValue kShooterSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kShooterSpinMotorProportional = 0;
    public static final double kShooterSpinMotorIntegral = 0;
    public static final double kShooterSpinMotorDerivative = 0;
    public static final double kShooterSpinMotorGravityFeedForward = 0;
    public static final double kShooterSpinMotorVelocityFeedForward = 0;
    public static final double kShooterSpinMotorStaticFeedForward = 0;

/*
 * ####################################################################################################################################
 * ####################################################### Indexer ###############################################################
 * ####################################################################################################################################
*/

    public static final double kIndexerMotorSupplyCurrentLimit = 0;
    public static final double kIndexerMotorClosedLoopRampPeriod = 0;
    public static final double kIndexerMotorPeakForwardVoltage = 0;
    public static final double kIndexerMotorPeakReverseVoltage = 0;
    public static final InvertedValue kIndexerMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kIndexerMotorProportional = 0;
    public static final double kIndexerMotorIntegral = 0;
    public static final double kIndexerMotorDerivative = 0;
    public static final double kIndexerMotorGravityFeedForward = 0;
    public static final double kIndexerMotorVelocityFeedForward = 0;
    public static final double kIndexerMotorStaticFeedForward = 0;

/*
 * ####################################################################################################################################
 * ####################################################### Hood ###############################################################
 * ####################################################################################################################################
*/

    public static final double kHoodMotorSupplyCurrentLimit = 0;
    public static final double kHoodMotorClosedLoopRampPeriod = 0;
    public static final double kHoodMotorPeakForwardVoltage = 0;
    public static final double kHoodMotorPeakReverseVoltage = 0;
    public static final InvertedValue kHoodMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kHoodMotorProportional = 0;
    public static final double kHoodMotorIntegral = 0;
    public static final double kHoodMotorDerivative = 0;
    public static final double kHoodMotorGravityFeedForward = 0;
    public static final double kHoodMotorVelocityFeedForward = 0;
    public static final double kHoodMotorStaticFeedForward = 0; 

/*
 * ####################################################################################################################################
 * ####################################################### Turret ###############################################################
 * ####################################################################################################################################
*/

    public static final double kTurretMotorSupplyCurrentLimit = 0;
    public static final double kTurretMotorClosedLoopRampPeriod = 0;
    public static final double kTurretMotorPeakForwardVoltage = 0;
    public static final double kTurretMotorPeakReverseVoltage = 0;
    public static final InvertedValue kTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kTurretMotorProportional = 0;
    public static final double kTurretMotorIntegral = 0;
    public static final double kTurretMotorDerivative = 0;
    public static final double kTurretMotorGravityFeedForward = 0;
    public static final double kTurretMotorVelocityFeedForward = 0;
    public static final double kTurretMotorStaticFeedForward = 0;

/*
 * ####################################################################################################################################
 * ####################################################### Climber Spin ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimberSpinMotorSupplyCurrentLimit = 0;
    public static final double kClimberSpinMotorClosedLoopRampPeriod = 0;
    public static final double kClimberSpinMotorPeakForwardVoltage = 0;
    public static final double kClimberSpinMotorPeakReverseVoltage = 0;
    public static final InvertedValue kClimberSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimberSpinMotorProportional = 0;
    public static final double kClimberSpinMotorIntegral = 0;
    public static final double kClimberSpinMotorDerivative = 0;
    public static final double kClimberSpinMotorGravityFeedForward = 0;
    public static final double kClimberSpinMotorVelocityFeedForward = 0;
    public static final double kClimberSpinMotorStaticFeedForward = 0;
/*
 * ####################################################################################################################################
 * ####################################################### Climb Rotate ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimbRotateMotorSupplyCurrentLimit = 0;
    public static final double kClimbRotateMotorClosedLoopRampPeriod = 0;
    public static final double kClimbRotateMotorPeakForwardVoltage = 0;
    public static final double kClimbRotateMotorPeakReverseVoltage = 0;
    public static final InvertedValue kClimbRotateMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimbRotateMotorProportional = 0;
    public static final double kClimbRotateMotorIntegral = 0;
    public static final double kClimbRotateMotorDerivative = 0;
    public static final double kClimbRotateMotorGravityFeedForward = 0;
    public static final double kClimbRotateMotorVelocityFeedForward = 0;
    public static final double kClimbRotateMotorStaticFeedForward = 0;
}
