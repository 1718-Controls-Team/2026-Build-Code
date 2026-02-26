package frc.robot.commands;



import frc.robot.subsystems.intakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class retractIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intakeSubsystem m_intakeSubsystem;
  
    private boolean m_isFinished = false;  
    private int m_retract = 0;
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public retractIntake(intakeSubsystem intake) {
      m_intakeSubsystem = intake;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_retract = 1;

    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   switch (m_retract) {
    case 1:
      m_intakeSubsystem.setIntakeSpinSpeed(Constants.kIntakeInSpeed);
      m_retract = 2;
      break;
    case 2:
      if (m_intakeSubsystem.getIntakeSpinSpeed() >= (Constants.kIntakeInSpeed - 1)) {
        m_intakeSubsystem.setIntakeElectricSlidePos(Constants.kIntakeSlideInPos);
      }
      m_retract = 0;
      break;
   }
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