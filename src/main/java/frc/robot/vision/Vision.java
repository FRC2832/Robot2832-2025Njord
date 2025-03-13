package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.swervedrive.SwerveSubsystem;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.livoniawarriors.UtilFunctions;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class Vision extends SubsystemBase {
  /** April Tag Field Layout of the year. */
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /** Photon Vision Simulation */
  private VisionSystemSim visionSim;

  private SwerveSubsystem swerve;

  private ArrayList<AprilTagCamera> cameras;

  @AutoLogOutput double distTo12;
  @AutoLogOutput double distTo18;

  private HashMap<Poles, Pose2d> bluePoses = new HashMap<Vision.Poles, Pose2d>();
  private HashMap<Poles, Pose2d> redPoses = new HashMap<Vision.Poles, Pose2d>();

  public Vision(SwerveSubsystem swerve) {
    // register this subsystem with the command scheduler to have the periodic function called
    super();
    AutoLogOutputManager.addObject(this);
    this.swerve = swerve;

    // get blue poles
    bluePoses.put(Poles.PoleA, new Pose2d(3.180, 4.212, Rotation2d.fromDegrees(-1)));
    bluePoses.put(Poles.PoleB, new Pose2d(3.682, 3.013, Rotation2d.fromDegrees(-0.1)));
    bluePoses.put(Poles.PoleC, new Pose2d(3.643, 2.967, Rotation2d.fromDegrees(61)));
    bluePoses.put(Poles.PoleD, new Pose2d(3.953, 2.792, Rotation2d.fromDegrees(61)));
    bluePoses.put(Poles.PoleE, new Pose2d(5.008, 2.814, Rotation2d.fromDegrees(118.9)));
    bluePoses.put(Poles.PoleF, new Pose2d(5.306, 2.995, Rotation2d.fromDegrees(121)));
    bluePoses.put(Poles.PoleG, new Pose2d(5.795, 3.844, Rotation2d.fromDegrees(178.7)));
    bluePoses.put(Poles.PoleH, new Pose2d(5.792, 4.198, Rotation2d.fromDegrees(-179)));
    bluePoses.put(Poles.PoleI, new Pose2d(5.339, 5.071, Rotation2d.fromDegrees(-121)));
    bluePoses.put(Poles.PoleJ, new Pose2d(4.837, 5.324, Rotation2d.fromDegrees(-119.5)));
    bluePoses.put(Poles.PoleK, new Pose2d(3.984, 5.247, Rotation2d.fromDegrees(-61.3)));
    bluePoses.put(Poles.PoleL, new Pose2d(3.714, 5.105, Rotation2d.fromDegrees(-60)));

    // get red poles
    for (var pole : bluePoses.keySet()) {
      var test = flipFieldAlways(bluePoses.get(pole));
      redPoses.put(pole, test);
    }

    /*
        redPoses.put(Poles.PoleA, new Pose2d());
        redPoses.put(Poles.PoleB, new Pose2d());
        redPoses.put(Poles.PoleC, new Pose2d(3.643, 2.967, Rotation2d.fromDegrees(61)));
        redPoses.put(Poles.PoleD, new Pose2d(3.953, 2.792, Rotation2d.fromDegrees(61)));
        redPoses.put(Poles.PoleE, new Pose2d());
        redPoses.put(Poles.PoleF, new Pose2d(5.306, 2.995, Rotation2d.fromDegrees(121)));
        redPoses.put(Poles.PoleG, new Pose2d());
        redPoses.put(Poles.PoleH, new Pose2d(5.792, 4.198, Rotation2d.fromDegrees(-179)));
        redPoses.put(Poles.PoleI, new Pose2d(5.339, 5.071, Rotation2d.fromDegrees(-121)));
        redPoses.put(Poles.PoleJ, new Pose2d());
        redPoses.put(Poles.PoleK, new Pose2d());
        redPoses.put(Poles.PoleL, new Pose2d(3.714, 5.105, Rotation2d.fromDegrees(-60)));
    */
    init();
  }

  public void addCamera(AprilTagCamera camera) {
    cameras.add(camera);
    camera.addToVisionSim(visionSim);
  }

  private void init() {
    cameras = new ArrayList<>();

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the
   *     robot to position itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException(
          "Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  @Override
  public void periodic() {
    if (SwerveDriveTelemetry.isSimulation && swerve.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerve.getSimulationDriveTrainPose().get());
    }
    for (AprilTagCamera c : cameras) {
      Optional<EstimatedRobotPose> poseEst = updateCamera(c);
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        swerve.addVisionMeasurement(
            pose.estimatedPose.toPose2d(), pose.timestampSeconds, c.curStdDevs);
      }
    }

    distTo12 =
        UtilFunctions.getDistance(swerve.getPose(), fieldLayout.getTagPose(12).get().toPose2d());
    distTo18 =
        UtilFunctions.getDistance(swerve.getPose(), fieldLayout.getTagPose(18).get().toPose2d());
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   *
   * <ul>
   *   <li>No Pose Estimates could be generated
   *   <li>The generated pose estimate was considered not accurate
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to
   *     create the estimate
   */
  public Optional<EstimatedRobotPose> updateCamera(AprilTagCamera camera) {
    Optional<EstimatedRobotPose> poseEst = camera.updateCamera();
    if (Robot.isSimulation()) {
      // Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision position estimates in sim.
      // poseEst.ifPresentOrElse(
      //    est ->
      //        debugField
      //            .getObject("VisionEstimation")
      //            .setPose(est.estimatedPose.toPose2d()),
      //    () -> {
      //      debugField.getObject("VisionEstimation").setPoses();
      //    });
    } //
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(swerve.getPose(), pose3d.toPose2d()))
        .orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, AprilTagCamera camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;
  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on
   * localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      //      try
      //      {
      //        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      //        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      //        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      //      } catch (IOException | URISyntaxException e)
      //      {
      //        e.printStackTrace();
      //      }
    }
  }

  /** Update the {@link Field2d} to include tracked targets/ */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (AprilTagCamera c : cameras) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    swerve.getField2d().getObject("tracked targets").setPoses(poses);
  }

  public enum Poles {
    PoleA,
    PoleB,
    PoleC,
    PoleD,
    PoleE,
    PoleF,
    PoleG,
    PoleH,
    PoleI,
    PoleJ,
    PoleK,
    PoleL
  }

  public Pose2d getPoleLocation(Poles pole) {
    if (swerve.isRedAlliance()) {
      return redPoses.get(pole);
    } else {
      return bluePoses.get(pole);
    }
  }

  public Pose2d flipAlliance(Pose2d poseToFlip) {
    if (swerve.isRedAlliance()) {
      return poseToFlip.relativeTo(
          new Pose2d(
              new Translation2d(fieldLayout.getFieldLength(), fieldLayout.getFieldWidth()),
              new Rotation2d(Math.PI)));
    } else {
      return poseToFlip;
    }
  }

  /**
   * Always flips alliance, irregardless of alliance
   *
   * @param poseToFlip
   * @return
   */
  public Pose2d flipFieldAlways(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(
        new Pose2d(
            new Translation2d(fieldLayout.getFieldLength(), fieldLayout.getFieldWidth()),
            new Rotation2d(Math.PI)));
  }

  public HashMap<Poles, Pose2d> getPoles(boolean red) {
    if (red) {
      return redPoses;
    } else {
      return bluePoses;
    }
  }
}
