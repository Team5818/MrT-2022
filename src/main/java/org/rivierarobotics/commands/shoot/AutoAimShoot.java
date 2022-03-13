package org.rivierarobotics.commands.shoot;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.rivierarobotics.subsystems.shoot.Floppas;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;

public class AutoAimShoot extends SequentialCommandGroup {
    public AutoAimShoot(boolean isAuto) {
        addCommands(
                new ConditionalCommand(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                new WaitCommand(0.5),
                                new Shoot(Floppas.getInstance().getEstimatedSpeed(Limelight.getInstance().getDistance()), Floppas.getInstance().getEstimatedAngle(Limelight.getInstance().getDistance()))
                                ),
                        new TrackGoal(isAuto)
                        ),
                        new Shoot(135, Floppas.ZERO_ANGLE - 0.562),
                        () -> Limelight.getInstance().getDetected()
                )
        );
    }

    public AutoAimShoot() {
        this(false);
    }

    @Override
    public void end(boolean interrupted) {
        DriveTrain.getInstance().setUseDriverAssist(false);
    }
}
