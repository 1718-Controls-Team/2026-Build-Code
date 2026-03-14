package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;


public class Constants {
    public static final CANBus kCanivore = new CANBus("Canivore");
    public static final double[] kBlueHubCoord = {4.627, 4.024};
    public static final double[] kRedHubCoord = {4.627, 4.024};
    public static final double[] kBluePassCoords = {1.905, 6.222, 2.059};

    // key - distance    value - speed
    public static final InterpolatingDoubleTreeMap kSpeedTable = new InterpolatingDoubleTreeMap();
    static {
        kSpeedTable.put(3.51, 60.0);
        kSpeedTable.put(2.6, 50.0);
        kSpeedTable.put(1.8, 47.0);

    } 

    // This is only necessary for shooting on move
    // The value is the time from shooting to the time that the shot lands
    // key - distance    value - time till shot lands
    public static final InterpolatingDoubleTreeMap kShotTimeTable = new InterpolatingDoubleTreeMap();
    static {
        kShotTimeTable.put(3.66, 1.3);
        kShotTimeTable.put(2.74, 1.2);
    }

    // limelight forward pos according to intake position
    public static final InterpolatingDoubleTreeMap kLLForwardPos = new InterpolatingDoubleTreeMap();
    static {
        kLLForwardPos.put(0.0, 0.4699);
        kLLForwardPos.put(-2.0, 0.6199);
    }
    public static final double kAccelCompFactor = 0.1;

/*
 * ####################################################################################################################################
 * ##################################################### Positions ###################################################################
 * ####################################################################################################################################
*/

// Velocity Controls
    public static final double kIndexerMainSpeed = 50;
    public static final double kIndexerNoSpeed = 0;

    public static final double kShooterOutSpeed = 63;


    public static final double kRollerMainSpeed = 30;

    public static final double kIntakeInSpeed = -60;
    public static final double kIntakeIdleSpeed = -10;
    public static final double kIntakeNoSpeed = 0;

    public static final double kClimbUpSpeed = 30;

    


// Position Controls
    public static final double kClimbRotateOutPos = 13;
    public static final double kClimbRotateDownPos = 0;

    // Intake 0 -> -15
    public static final double kIntakeSlideOutPos = -14.6;
    public static final double kIntakeSlideInPos = -6.697;

    public static final double kHoodServoDownPos = 0.2;

    public static final double kTurretMax = .35;
    public static final double kTurretMin = -0.2;


/*
 * ####################################################################################################################################
 * ####################################################### Intake Spin ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kIntakeSpinMotorProportional = 2;
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

    public static final double kIntakeElectricSlideMotorProportional = 2;
    public static final double kIntakeElectricSlideMotorIntegral = 0;
    public static final double kIntakeElectricSlideMotorDerivative = 0;    

    public static final double kIntakeElectricSlideMotorVelocityFeedForward = 0.2;
    public static final double kIntakeElectricSlideMotorGravityFeedForward = 0;

    public static final double kIntakeElectricSlideMotorMaxForwardVoltage = 11;
    public static final double kIntakeElectricSlideMotorMaxReverseVoltage = -11;

    public static final double kIntakeElectricSlideMotorVoltageClosedLoopRampPeriod = 0;
    public static final InvertedValue kIntakeElectricSlideMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kIntakeElectricSlideMotorSupplyCurrentLimit = 20;
    


/*
 * ####################################################################################################################################
 * ####################################################### Right Roller ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kRightRollerSpinMotorProportional = 0;
    public static final double kRightRollerSpinMotorIntegral = 0;
    public static final double kRightRollerSpinMotorDerivative = 0;

    public static final double kRightRollerSpinMotorVelocityFeedForward = 0.122;
    public static final double kRightRollerSpinMotorStaticFeedForward = 0;
    public static final double kRightRollerSpinMotorGravityFeedForward = 0;

    public static final double kRightRollerSpinMotorPeakForwardVoltage = 11;
    public static final double kRightRollerSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightRollerSpinMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kRightRollerSpinMotorSupplyCurrentLimit = 40;

    public static final double kRightRollerSpinMotorClosedLoopRampPeriod = 0;
/*
 * ####################################################################################################################################
 * ####################################################### Left Roller ###############################################################
 * ####################################################################################################################################
*/
    
    public static final double kLeftRollerSpinMotorProportional = 0;
    public static final double kLeftRollerSpinMotorIntegral = 0;
    public static final double kLeftRollerSpinMotorDerivative = 0;

    public static final double kLeftRollerSpinMotorVelocityFeedForward = 0.122;
    public static final double kLeftRollerSpinMotorStaticFeedForward = 0;
    public static final double kLeftRollerSpinMotorGravityFeedForward = 0;

    public static final double kLeftRollerSpinMotorPeakForwardVoltage = 11;
    public static final double kLeftRollerSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftRollerSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftRollerSpinMotorSupplyCurrentLimit = 40;

    public static final double kLeftRollerSpinMotorClosedLoopRampPeriod = 0;
    
      
  
/*
 * ####################################################################################################################################
 * ####################################################### Shooter Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kLeftShooterMotorProportional = 3;
    public static final double kLeftShooterMotorIntegral = 0;
    public static final double kLeftShooterMotorDerivative = 0.1;

    public static final double kLeftShooterMotorGravityFeedForward = 0;
    public static final double kLeftShooterMotorVelocityFeedForward = 1.2;
    public static final double kLeftShooterMotorStaticFeedForward = 0;

    public static final double kLeftShooterMotorPeakForwardVoltage = 11;
    public static final double kLeftShooterMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftShooterMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kLeftShooterMotorSupplyCurrentLimit = 40;

    public static final double kLeftShooterMotorClosedLoopRampPeriod = 0;
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Shooter Right ###############################################################
 * ####################################################################################################################################
*/

    public static final double kRightShooterMotorProportional = 3;
    public static final double kRightShooterMotorIntegral = 0;
    public static final double kRightShooterMotorDerivative = 0.1;

    public static final double kRightShooterMotorGravityFeedForward = 0;
    public static final double kRightShooterMotorVelocityFeedForward = 1.2;
    public static final double kRightShooterMotorStaticFeedForward = 0;

    public static final double kRightShooterMotorPeakForwardVoltage = 11;
    public static final double kRightShooterMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightShooterMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kRightShooterMotorSupplyCurrentLimit = 40;

    public static final double kRightShooterMotorClosedLoopRampPeriod = 0;
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Indexer Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kLeftIndexerMotorProportional = 3;
    public static final double kLeftIndexerMotorIntegral = 0;
    public static final double kLeftIndexerMotorDerivative = 0;

    public static final double kLeftIndexerMotorGravityFeedForward = 0;
    public static final double kLeftIndexerMotorVelocityFeedForward = 0.112;
    public static final double kLeftIndexerMotorStaticFeedForward = 0;

    public static final double kLeftIndexerMotorPeakForwardVoltage = 11;
    public static final double kLeftIndexerMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftIndexerMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftIndexerMotorSupplyCurrentLimit = 40;

    public static final double kLeftIndexerMotorClosedLoopRampPeriod = 0;
    
/*
 * ####################################################################################################################################
 * ####################################################### Indexer Right ###############################################################
 * ####################################################################################################################################
*/

    public static final double kRightIndexerMotorProportional = 3;
    public static final double kRightIndexerMotorIntegral = 0;
    public static final double kRightIndexerMotorDerivative = 0;

    public static final double kRightIndexerMotorGravityFeedForward = 0;
    public static final double kRightIndexerMotorVelocityFeedForward = 0.112;
    public static final double kRightIndexerMotorStaticFeedForward = 0;

    public static final double kRightIndexerMotorPeakForwardVoltage = 11;
    public static final double kRightIndexerMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightIndexerMotorDirection = InvertedValue.CounterClockwise_Positive;
    public static final double kRightIndexerMotorSupplyCurrentLimit = 40;

    public static final double kRightIndexerMotorClosedLoopRampPeriod = 0;
    

/*
 * ####################################################################################################################################
 * ####################################################### Turret Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kLeftTurretMotorProportional = 10;
    public static final double kLeftTurretMotorIntegral = 0;
    public static final double kLeftTurretMotorDerivative = 0.3;

    public static final double kLeftTurretMotorGravityFeedForward = 0;
    public static final double kLeftTurretMotorVelocityFeedForward = 0.2;
    
    public static final double kLeftTurretMotorPeakForwardVoltage = 11;
    public static final double kLeftTurretMotorPeakReverseVoltage = -11;

    public static final InvertedValue kLeftTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kLeftTurretMotorSupplyCurrentLimit = 40;

    public static final double kLeftTurretMotorClosedLoopRampPeriod = 0;
    
/*
 * ####################################################################################################################################
 * ####################################################### Turret Left ###############################################################
 * ####################################################################################################################################
*/

    public static final double kRightTurretMotorProportional = 10;
    public static final double kRightTurretMotorIntegral = 0;
    public static final double kRightTurretMotorDerivative = 0.3;

    public static final double kRightTurretMotorGravityFeedForward = 0;
    public static final double kRightTurretMotorVelocityFeedForward = 0.2;
    
    public static final double kRightTurretMotorPeakForwardVoltage = 11;
    public static final double kRightTurretMotorPeakReverseVoltage = -11;

    public static final InvertedValue kRightTurretMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kRightTurretMotorSupplyCurrentLimit = 40;

    public static final double kRightTurretMotorClosedLoopRampPeriod = 0;
    

/*
 * ####################################################################################################################################
 * ####################################################### Climber Spin ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimberSpinMotorProportional = 0;
    public static final double kClimberSpinMotorIntegral = 0;
    public static final double kClimberSpinMotorDerivative = 0;
    
    public static final double kClimberSpinMotorGravityFeedForward = 0;
    public static final double kClimberSpinMotorVelocityFeedForward = 0.112;
    public static final double kClimberSpinMotorStaticFeedForward = 0;

    public static final double kClimberSpinMotorPeakForwardVoltage = 11;
    public static final double kClimberSpinMotorPeakReverseVoltage = -11;

    public static final InvertedValue kClimberSpinMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimberSpinMotorSupplyCurrentLimit = 40;

    public static final double kClimberSpinMotorClosedLoopRampPeriod = 0;
    
    
    
/*
 * ####################################################################################################################################
 * ####################################################### Climb Rotate ###############################################################
 * ####################################################################################################################################
*/

    public static final double kClimbRotateMotorProportional = 0;
    public static final double kClimbRotateMotorIntegral = 0;
    public static final double kClimbRotateMotorDerivative = 0;

    public static final double kClimbRotateMotorGravityFeedForward = 0;
    public static final double kClimbRotateMotorVelocityFeedForward = 0.2;

    public static final double kClimbRotateMotorPeakForwardVoltage = 11;
    public static final double kClimbRotateMotorPeakReverseVoltage = -11;

    public static final InvertedValue kClimbRotateMotorDirection = InvertedValue.Clockwise_Positive;
    public static final double kClimbRotateMotorSupplyCurrentLimit = 40;

    public static final double kClimbRotateMotorClosedLoopRampPeriod = 0;
    
    
    
}
