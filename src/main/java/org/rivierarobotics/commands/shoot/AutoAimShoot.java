package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Floppas;
import org.rivierarobotics.subsystems.vision.Limelight;

public class AutoAimShoot extends SequentialCommandGroup {
    public AutoAimShoot() {
        addCommands(
                new ConditionalCommand(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new Shoot(Floppas.getInstance().getEstimatedSpeed(Limelight.getInstance().getDistance()), Floppas.getInstance().getEstimatedAngle(Limelight.getInstance().getDistance()))
                                ),
                                new TrackGoal()
                        ), null, () -> Limelight.getInstance().getDetected()
                )

        );
    }
}
