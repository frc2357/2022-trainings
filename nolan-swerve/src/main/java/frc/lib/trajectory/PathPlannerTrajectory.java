package frc.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class PathPlannerTrajectory extends Trajectory {
    protected PathPlannerTrajectory(ArrayList<Waypoint> pathPoints, double maxVel, double maxAccel, boolean reversed) {
        super(generatePath(pathPoints, maxVel, maxAccel, reversed));
    }

    protected PathPlannerTrajectory(List<State> states) {
        super(states);
    }

    @Override
    public State sample(double time) {
        if
    }


    protected static class Waypoint {
        private final Translation2d anchorPoint;
        private final Translation2d prevControl;
        private final Translation2d nextControl;
        private final double velOverride;
        private final Rotation2d holonomicRotation;
        protected final boolean isReversal;

        protected Waypoint(Translation2d anchorPoint, Translation2d prevControl, Translation2d nextControl, double velOverride, Rotation2d holonomicRotation, boolean isReversal) {
            this.anchorPoint = anchorPoint;
            this.prevControl = prevControl;
            this.nextControl = nextControl;
            this.velOverride = velOverride;
            this.holonomicRotation = holonomicRotation;
            this.isReversal = isReversal;
        }
    }

}
