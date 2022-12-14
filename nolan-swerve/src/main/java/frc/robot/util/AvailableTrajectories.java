package frc.robot.util;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AvailableTrajectories {
    public static SequentialCommandGroup exampleTrajectory = null;
    public static SequentialCommandGroup exampleRecordPathTrajectory = null;
    public static SequentialCommandGroup leaveTarmacTrajectory = null;
    public static SequentialCommandGroup travelToThirdCargoTrajectory = null;
    public static SequentialCommandGroup travelToTerminalTrajectory = null;

    public static void generateTrajectories() {
        exampleTrajectory = createExampleTrajectory();
        exampleRecordPathTrajectory = createExampleRecordPathTrajectory();
        leaveTarmacTrajectory = createLeaveTarmacTrajectory();
        travelToThirdCargoTrajectory = createTravelToThirdCargoTrajectory();
        travelToTerminalTrajectory = createTravelToTerminalTrajectory();
    }

    private static SequentialCommandGroup createExampleTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(DrivetrainSubsystem.getInstance(),
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0.0,0.0)),
                new Pose2d(3, 0, new Rotation2d(0)), false, true);
    }

    private static SequentialCommandGroup createExampleRecordPathTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(DrivetrainSubsystem.getInstance(),
                List.of(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(0.012195453405645025, -1.8901838807378317E-6, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(0.173697545532843, -2.0990006162902373E-5, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(0.5976884276133373, -0.06595781469014239, Rotation2d.fromDegrees(-30.849609375)),
                        new Pose2d(0.9299569878347765, -0.3704943275575454, Rotation2d.fromDegrees(-44.208984375)),
                        new Pose2d(1.3588293482304472, -0.7731543346116618, Rotation2d.fromDegrees(-31.025390625)),
                        new Pose2d(1.7672559181468923, -0.5581337045202731, Rotation2d.fromDegrees(81.12304687500001)),
                        new Pose2d(1.8516981992006087, 0.04452195822728959, Rotation2d.fromDegrees(81.9580078125)),
                        new Pose2d(1.9765518341234347, 0.6303516511838099, Rotation2d.fromDegrees(62.8857421875)),
                        new Pose2d(2.4403234462286174, 0.8288020947695366, Rotation2d.fromDegrees(-27.158203125000004)),
                        new Pose2d(2.919374962567724, 0.4728429080958283, Rotation2d.fromDegrees(-36.1669921875)),
                        new Pose2d(3.431510890857734, 0.2802894687801234, Rotation2d.fromDegrees(-0.9667968750000001)),
                        new Pose2d(3.451430118783809, 0.2801714314001176, Rotation2d.fromDegrees(0.0))),
                false, true);
    }

    private static SequentialCommandGroup createLeaveTarmacTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(DrivetrainSubsystem.getInstance(),
                List.of(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(0.017513853118839913, -4.738528291391344E-5,
                                Rotation2d.fromDegrees(-0.13183593750000003)),
                        new Pose2d(0.3417937931856386, 0.003688144413647491, Rotation2d.fromDegrees(1.494140625)),
                        new Pose2d(0.8130550100571042, 0.01402617100189901, Rotation2d.fromDegrees(1.0546875000000002)),
                        new Pose2d(0.9625838482837916, 0.016549502356075444,
                                Rotation2d.fromDegrees(0.30761718749999994))),
                false, true);
    }

    private static SequentialCommandGroup createTravelToThirdCargoTrajectory() {
        return new SequentialCommandGroup(createThreeBallTravelBackTrajectory(),
                createThreeBallAutoTravelToThirdTrajectory());
    }

    private static SequentialCommandGroup createThreeBallTravelBackTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(
                DrivetrainSubsystem.getInstance(), List.of(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                        new Pose2d(-0.319070674890391, 0.01856578744447176, Rotation2d.fromDegrees(-8.0419921875)),
                        new Pose2d(-1.3102390255302991, 0.5075593681858235,
                                Rotation2d.fromDegrees(-48.03222656249999))),
                true, true);
    }

    private static SequentialCommandGroup createThreeBallAutoTravelToThirdTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(DrivetrainSubsystem.getInstance(), List.of(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(0.2650097089203349, -0.029321698868923312, Rotation2d.fromDegrees(-12.12890625)),
                new Pose2d(1.1462756519915793, -0.47026944957780487, Rotation2d.fromDegrees(-42.0556640625)),
                new Pose2d(1.6972924521141421, -1.2178663849396316, Rotation2d.fromDegrees(-62.666015625000014)),
                new Pose2d(1.8826092475307807, -2.000361663170719003, Rotation2d.fromDegrees(-67.63183593750001))),
                false, true);
    }

    private static SequentialCommandGroup createTravelToTerminalTrajectory() {
        return TrajectoryUtil.createTrajectoryPathCommand(DrivetrainSubsystem.getInstance(),
        List.of( new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        new Pose2d(0.3941938656014967, -0.004898079102755263, Rotation2d.fromDegrees(-2.4169921875)),
        new Pose2d(1.6588001849430754, -0.15035216099877424, Rotation2d.fromDegrees(-8.876953125000002)),
        new Pose2d(3.044482693701453, -0.3806636740437954, Rotation2d.fromDegrees(-9.7119140625)),
        new Pose2d(3.4151729881570407, -0.4438813539688769, Rotation2d.fromDegrees(-8.745117187500002))),
        false, true);

    }

    /**
     List.of( new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
new Pose2d(0.3941938656014967, -0.004898079102755263, Rotation2d.fromDegrees(-2.4169921875)),
new Pose2d(1.6588001849430754, -0.15035216099877424, Rotation2d.fromDegrees(-8.876953125000002)),
new Pose2d(3.044482693701453, -0.3806636740437954, Rotation2d.fromDegrees(-9.7119140625)),
new Pose2d(3.4151729881570407, -0.4438813539688769, Rotation2d.fromDegrees(-8.745117187500002))),
     */

    /**
     * List.of( new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
new Pose2d(0.32343865987046494, 0.029636948658706898, Rotation2d.fromDegrees(11.865234374999998)),
new Pose2d(1.1061287309008285, 0.43835817722185644, Rotation2d.fromDegrees(42.4072265625)),
new Pose2d(1.6790318724922213, 1.3030251606772916, Rotation2d.fromDegrees(66.6650390625)),
new Pose2d(2.011971662567233, 2.4178732487834784, Rotation2d.fromDegrees(80.1123046875)),
new Pose2d(2.0803038696467513, 3.2219424299446793, Rotation2d.fromDegrees(90.2197265625))),
     */

    /**
     * List.of( new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
new Pose2d(0.36864890537395567, 0.050704275604839324, Rotation2d.fromDegrees(15.1171875)),
new Pose2d(1.3387126682076886, 0.7124265987918048, Rotation2d.fromDegrees(51.240234375)),
new Pose2d(1.7942236920981458, 1.9681594451819513, Rotation2d.fromDegrees(83.8916015625)),
new Pose2d(1.855321520478801, 3.1558796336807435, Rotation2d.fromDegrees(88.681640625)),
new Pose2d(1.8571553351584495, 3.258267650622096, Rotation2d.fromDegrees(88.41796875))),
     */

    /**
     * List.of( new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
new Pose2d(0.3452392687824499, 0.045784218451575306, Rotation2d.fromDegrees(16.9189453125)),
new Pose2d(1.0862206555512335, 0.7135699625763575, Rotation2d.fromDegrees(62.797851562500014)),
new Pose2d(1.329158562887138, 1.8840473081707785, Rotation2d.fromDegrees(89.47265625)),
new Pose2d(1.213637317029497, 3.6, Rotation2d.fromDegrees(94.2626953125))),
     */
}