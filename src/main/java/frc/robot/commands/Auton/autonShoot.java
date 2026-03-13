// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import frc.robot.subsystems.shooterIndexer;
import frc.robot.subsystems.intakeFuel;
import frc.robot.subsystems.spiralRoller;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class autonShoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shooterIndexer m_shooterSubsystem;
  private final spiralRoller m_spiralRollerSubsystem;
  private final intakeFuel m_intakeSubsystem;
  
  
    private boolean m_isFinished = false;
    private int shootFlag = 0;
    Timer spiralTimer = new Timer();
  
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
      public autonShoot(shooterIndexer shooter, spiralRoller spirals, intakeFuel intake) {
        m_shooterSubsystem = shooter;
        m_spiralRollerSubsystem = spirals;
        m_intakeSubsystem = intake;
    
      addRequirements(shooter);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shootFlag = 1;

      
      
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shootFlag) {
        case 1:
            m_spiralRollerSubsystem.setSpiralRollerSpinSpeed(Constants.kRollerMainSpeed);
            m_shooterSubsystem.setShooterSpinSpeed(Constants.kShooterOutSpeed);
            spiralTimer.reset();
            spiralTimer.start();
            shootFlag = 2;
          break;
        case 2:
          if (spiralTimer.get() >= 0.5) {
            m_shooterSubsystem.setIndexerSpinSpeed(Constants.kIndexerMainSpeed);
            shootFlag = 3;
          }
          break; 
        case 3:
          for (int i = 0; i < 5; i++) {
            if (m_intakeSubsystem.getIntakeElectricSlidePos() > -7) {
              m_intakeSubsystem.setIntakeElectricSlidePos(Constants.kIntakeSlideOutPos);
              spiralTimer.reset();
              spiralTimer.start();
            }
            if (spiralTimer.get() >= .5) {
              m_intakeSubsystem.setIntakeElectricSlidePos(Constants.kIntakeSlideInPos);
            }
          }
          break;

      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  m_shooterSubsystem.setShooterOff(0);
    m_spiralRollerSubsystem.setSpiralRollerOff(0);
    m_intakeSubsystem.setIntakeOutput(0);
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
