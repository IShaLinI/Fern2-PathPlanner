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
  private CommandXboxController mTam = new CommandXboxController(2);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {


    //Change based on driver
    // mDrivetrain.setDefaultCommand(
    //   new RunCommand(()-> mDrivetrain.drive(
    //     Deadbander.applyLinearScaledDeadband(mTam.getLeftX(),0.1) * DriveConstants.kMaxSpeed,
    //     Deadbander.applyLinearScaledDeadband(mTam.getRightY(), 0.1) * DriveConstants.kMaxTurnSpeed,
    //     mTam.leftTrigger().getAsBoolean()
    //   ),
    //   mDrivetrain
    // )
    // );

    mDrivetrain.setDefaultCommand(
      new RunCommand(()-> mDrivetrain.drive(
        Deadbander.applyLinearScaledDeadband(mDriver.getRightX(),0.1) * DriveConstants.kMaxSpeed,
        Deadbander.applyLinearScaledDeadband(mDriver.getLeftY(), 0.1) * DriveConstants.kMaxTurnSpeed,
        mDriver.leftTrigger().getAsBoolean()
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

    mTam.leftBumper().onTrue(
      new InstantCommand(mDrivetrain::toggleState)
    );

    mTam.b().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.SUBSTATION),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    );

    mTam.b().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mTam.x().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L1),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.L1RELEASE)
      )
    );
    
    mTam.x().onFalse(
      new ParallelCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mTam.y().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.L2),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.L2RELEASE)
      )
    );

    mTam.y().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
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
    );

    mOperator.x().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.STOP),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.rightBumper().whileTrue(
      new SequentialCommandGroup(
        mPivot.changeState(PivotConstants.State.FLOOR),
        new WaitUntilCommand(() -> mPivot.atTarget()),
        mIntake.changeState(IntakeConstants.State.GRAB)
      )
    );

    mOperator.rightBumper().onFalse(
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
    );
    
    mOperator.y().onFalse(
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
    );

    mOperator.b().onFalse(
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
    );

    mOperator.a().onFalse(
      new SequentialCommandGroup(
        mIntake.changeState(IntakeConstants.State.IDLE),
        mPivot.changeState(PivotConstants.State.CARRY)
      )
    );

    mOperator.rightTrigger().onTrue(
      mIntake.changeState(IntakeConstants.State.GRAB)
    );

    mOperator.rightTrigger().onFalse(
      mIntake.changeState(IntakeConstants.State.STOP)
    );

    mOperator.leftTrigger().onTrue(
      mIntake.changeState(IntakeConstants.State.RELEASE)
    );

    mOperator.leftTrigger().onFalse(
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

    return mDrivetrain.new ChargeStationAuto();

  }
}