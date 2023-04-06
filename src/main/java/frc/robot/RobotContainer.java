// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.controls.Deadbander;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class RobotContainer {

  private final Drivetrain mDrivetrain = new Drivetrain();
  private final Intake mIntake = new Intake();
  private final Pivot mPivot = new Pivot();

  private AutoCommands mAutoCommands = new AutoCommands(mDrivetrain, mIntake, mPivot);

  private CommandXboxController mDriver = new CommandXboxController(0);
  private CommandXboxController mOperator = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    mDrivetrain.setDefaultCommand(
      new RunCommand(()-> mDrivetrain.drive(
        Deadbander.applyLinearScaledDeadband(mDriver.getRightX(),0.1) * DriveConstants.kMaxSpeed,
        Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1) * DriveConstants.kMaxTurnSpeed,
        mDriver.leftTrigger().getAsBoolean(),
        mDriver.rightTrigger().getAsBoolean()
      ),
      mDrivetrain
    )
    );


    mDriver.povLeft().onTrue(
      new InstantCommand(
        mDrivetrain::resetEncoders)
    );
    
    mDriver.povUp().onTrue(
      mDrivetrain.changeState(DriveConstants.FrontState.FORWARD)
    );

    mDriver.povDown().onTrue(
      mDrivetrain.changeState(DriveConstants.FrontState.REVERSE)
    );

    mOperator.povRight().onTrue(
      new InstantCommand(
        mPivot::zeroEncoder,
        mPivot
      )
    );
    
    mOperator.x().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.SUBSTATION),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    ).whileFalse(
        new SequentialCommandGroup(
          mIntake.changeState(IntakeConstants.State.STOP),
          mPivot.changeState(PivotConstants.State.CARRY))
    );
  

    mOperator.rightBumper().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.FLOOR),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    ).whileFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );
    
    mOperator.y().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L1),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.L1RELEASE)
      )
    ).whileFalse(
      new ParallelCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.b().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L2),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.L2RELEASE)
      )
    ).whileFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.a().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L3),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.L3RELEASE)
      )
    ).whileFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.rightTrigger().whileTrue(
      mIntake.changeState(IntakeConstants.State.GRAB)
    ).whileFalse(
      mIntake.changeState(IntakeConstants.State.STOP)
    );

   
    mOperator.leftTrigger().onTrue(
      mIntake.changeState(IntakeConstants.State.RELEASE)
    ).whileFalse(
      mIntake.changeState(IntakeConstants.State.STOP)
    );

    mOperator.povUp().onTrue(
      mPivot.changeState(PivotConstants.State.L2)
    );

    mOperator.povDown().onTrue(
      mPivot.changeState(PivotConstants.State.L1)
    );

  }

  public Command getAutonomousCommand() {

    return mAutoCommands.getAuto();

  }
}