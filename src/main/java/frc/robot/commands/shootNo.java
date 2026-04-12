// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.hoodServo;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.spiralRoller;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class shootNo extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final intakeFuel m_intakeSubsystem;
  private final hoodServo m_hoodSubsystem;
  
  
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    Timer spiralTimer = new Timer();
  
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public shootNo(shooterIndexer shooter, spiralRoller spirals, intakeFuel intake, hoodServo hood) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_intakeSubsystem = intake;
        m_hoodSubsystem = hood;
    
      addRequirements(shooter);
      addRequirements(spirals);
      addRequirements(intake);

    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    m_shooterSubsystem.setShooterOff(0);
    m_spiralRollerSubsystem.setSpiralRollerOff(0);
    m_intakeSubsystem.setIntakeOutput(0);
      
    }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
