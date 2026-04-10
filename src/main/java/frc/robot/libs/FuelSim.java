package frc.robot.libs;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FuelSim {
  protected static final double PERIOD = 0.02; // sec
  protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
                                                                                          // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
  protected static final double AIR_DENSITY = 1.2041; // kg/m^3
  protected static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with the field
  protected static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
  protected static final double NET_COR = 0.2; // coefficient of restitution with the net
  protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
  protected static final double FUEL_RADIUS = 0.075;
  protected static final double FIELD_LENGTH = 16.51;
  protected static final double FIELD_WIDTH = 8.04;
  protected static final double TRENCH_WIDTH = 1.265;
  protected static final double TRENCH_BLOCK_WIDTH = 0.305;
  protected static final double TRENCH_HEIGHT = 0.565;
  protected static final double TRENCH_BAR_HEIGHT = 0.102;
  protected static final double TRENCH_BAR_WIDTH = 0.152;
  protected static final double FRICTION = 0.1; // proportion of horizontal vel to lose per sec while on ground
  protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
  protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
  // Drag coefficient of smooth sphere: https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
  protected static final double DRAG_COF = 0.47; // dimensionless
  protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;
  protected static final double FUEL_RADIUS_SQ = FUEL_RADIUS * FUEL_RADIUS;
  protected static final double FUEL_DIAMETER = FUEL_RADIUS * 2;
  protected static final double FUEL_DIAMETER_SQ = FUEL_DIAMETER * FUEL_DIAMETER;
  protected static final Translation3d ZERO_TRANSLATION = new Translation3d();
  protected static final int STARTING_FUEL_COUNT = 15 * 6 * 4 + 3 * 4 * 4;
  protected static final double CENTER_FUEL_X_START = 0.076;
  protected static final double CENTER_FUEL_Y_START = 0.0254 + 0.076;
  protected static final double CENTER_FUEL_SPACING = 0.152;
  protected static final double DEPOT_FUEL_X_START = 0.076;
  protected static final double DEPOT_FUEL_OFFSET_START = 0.076;
  protected static final double DEPOT_FUEL_Y_LEFT_CENTER = 5.95;
  protected static final double DEPOT_FUEL_Y_RIGHT_CENTER = 2.09;
  protected static final double DEPOT_FUEL_SPACING = 0.152;

  protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
    new Translation3d(0, 0, 0),
    new Translation3d(3.96, 1.57, 0),
    new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(4.61, 1.57, 0.165),
    new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
    new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
    new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final class XZLineSegment {
    protected final double startX;
    protected final double startY;
    protected final double startZ;
    protected final double endY;
    protected final double vecX;
    protected final double vecZ;
    protected final double lengthSq;
    protected final double normalX;
    protected final double normalZ;

    protected XZLineSegment(Translation3d start, Translation3d end) {
      startX = start.getX();
      startY = start.getY();
      startZ = start.getZ();
      endY = end.getY();
      vecX = end.getX() - startX;
      vecZ = end.getZ() - startZ;
      lengthSq = vecX * vecX + vecZ * vecZ;

      double length = Math.sqrt(lengthSq);
      normalX = -vecZ / length;
      normalZ = vecX / length;
    }
  }

  protected static final XZLineSegment[] FIELD_XZ_LINES = new XZLineSegment[FIELD_XZ_LINE_STARTS.length];

  protected static final Translation3d[] TRENCH_RECT_STARTS = {
    new Translation3d(3.96, TRENCH_WIDTH, 0),
    new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
  };

  protected static final Translation3d[] TRENCH_RECT_ENDS = {
    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final Translation3d[] STARTING_FUEL_POSITIONS = createStartingFuelPositions();

  protected static Translation3d[] createStartingFuelPositions() {
    Translation3d[] positions = new Translation3d[STARTING_FUEL_COUNT];
    int index = 0;

    double centerX = FIELD_LENGTH / 2;
    double centerY = FIELD_WIDTH / 2;

    for (int i = 0; i < 15; i++) {
      double dy = CENTER_FUEL_Y_START + CENTER_FUEL_SPACING * i;
      for (int j = 0; j < 6; j++) {
        double dx = CENTER_FUEL_X_START + CENTER_FUEL_SPACING * j;
        positions[index++] = new Translation3d(centerX + dx, centerY + dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(centerX - dx, centerY + dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(centerX + dx, centerY - dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(centerX - dx, centerY - dy, FUEL_RADIUS);
      }
    }

    for (int i = 0; i < 3; i++) {
      double dy = DEPOT_FUEL_OFFSET_START + DEPOT_FUEL_SPACING * i;
      for (int j = 0; j < 4; j++) {
        double x = DEPOT_FUEL_X_START + DEPOT_FUEL_SPACING * j;
        positions[index++] = new Translation3d(x, DEPOT_FUEL_Y_LEFT_CENTER + dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(x, DEPOT_FUEL_Y_LEFT_CENTER - dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(FIELD_LENGTH - x, DEPOT_FUEL_Y_RIGHT_CENTER + dy, FUEL_RADIUS);
        positions[index++] = new Translation3d(FIELD_LENGTH - x, DEPOT_FUEL_Y_RIGHT_CENTER - dy, FUEL_RADIUS);
      }
    }

    return positions;
  }

  static {
    for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
      FIELD_XZ_LINES[i] = new XZLineSegment(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
    }
  }

  protected static class Fuel {
    protected Translation3d pos;
    protected Translation3d vel;

    protected Fuel(Translation3d pos, Translation3d vel) {
      this.pos = pos;
      this.vel = vel;
    }

    protected Fuel(Translation3d pos) {
      this(pos, ZERO_TRANSLATION);
    }

    protected void reset(Translation3d pos, Translation3d vel) {
      this.pos = pos;
      this.vel = vel;
    }

    protected void update(boolean simulateAirResistance, int subticks) {
      double dt = PERIOD / subticks;

      double px = pos.getX();
      double py = pos.getY();
      double pz = pos.getZ();
      double vx = vel.getX();
      double vy = vel.getY();
      double vz = vel.getZ();

      px += vx * dt;
      py += vy * dt;
      pz += vz * dt;

      if (pz > FUEL_RADIUS) {
        double ax = 0;
        double ay = 0;
        double az = GRAVITY.getZ();

        if (simulateAirResistance) {
          double speedSq = vx * vx + vy * vy + vz * vz;
          if (speedSq > 1e-12) {
            double speed = Math.sqrt(speedSq);
            double dragScale = -(DRAG_FORCE_FACTOR / FUEL_MASS) * speed;
            ax += vx * dragScale;
            ay += vy * dragScale;
            az += vz * dragScale;
          }
        }

        vx += ax * dt;
        vy += ay * dt;
        vz += az * dt;
      }

      if (Math.abs(vz) < 0.05 && pz <= FUEL_RADIUS + 0.03) {
        vz = 0;
        double frictionScale = 1 - FRICTION * dt;
        vx *= frictionScale;
        vy *= frictionScale;
      }

      pos = new Translation3d(px, py, pz);
      vel = new Translation3d(vx, vy, vz);
      handleFieldCollisions(subticks);
    }

    protected void handleXZLineCollision(XZLineSegment line) {
      double px = pos.getX();
      double py = pos.getY();
      double pz = pos.getZ();

      if (py < line.startY || py > line.endY) return;

      double fromStartX = px - line.startX;
      double fromStartZ = pz - line.startZ;
      double t = (fromStartX * line.vecX + fromStartZ * line.vecZ) / line.lengthSq;
      if (t < 0 || t > 1) return;

      double closestX = line.startX + line.vecX * t;
      double closestZ = line.startZ + line.vecZ * t;

      double sepX = px - closestX;
      double sepZ = pz - closestZ;
      double distSq = sepX * sepX + sepZ * sepZ;
      if (distSq > FUEL_RADIUS_SQ) return;

      double dist = Math.sqrt(distSq);
      double penetration = FUEL_RADIUS - dist;

      px += line.normalX * penetration;
      pz += line.normalZ * penetration;

      double vx = vel.getX();
      double vy = vel.getY();
      double vz = vel.getZ();
      double velAlongNormal = vx * line.normalX + vz * line.normalZ;

      if (velAlongNormal <= 0) {
        double response = (1 + FIELD_COR) * velAlongNormal;
        vx -= line.normalX * response;
        vz -= line.normalZ * response;
      }

      pos = new Translation3d(px, py, pz);
      vel = new Translation3d(vx, vy, vz);
    }

    protected void handleFieldCollisions(int subticks) {
      // floor and bumps
      for (int i = 0; i < FIELD_XZ_LINES.length; i++) {
        handleXZLineCollision(FIELD_XZ_LINES[i]);
      }

      double px = pos.getX();
      double py = pos.getY();
      double pz = pos.getZ();
      double vx = vel.getX();
      double vy = vel.getY();
      double vz = vel.getZ();

      // edges
      if (px < FUEL_RADIUS && vx < 0) {
        px += FUEL_RADIUS - px;
        vx += -(1 + FIELD_COR) * vx;
      } else if (px > FIELD_LENGTH - FUEL_RADIUS && vx > 0) {
        px += FIELD_LENGTH - FUEL_RADIUS - px;
        vx += -(1 + FIELD_COR) * vx;
      }

      if (py < FUEL_RADIUS && vy < 0) {
        py += FUEL_RADIUS - py;
        vy += -(1 + FIELD_COR) * vy;
      } else if (py > FIELD_WIDTH - FUEL_RADIUS && vy > 0) {
        py += FIELD_WIDTH - FUEL_RADIUS - py;
        vy += -(1 + FIELD_COR) * vy;
      }

      pos = new Translation3d(px, py, pz);
      vel = new Translation3d(vx, vy, vz);

      // hubs
      handleHubCollisions(Hub.BLUE_HUB, subticks);
      handleHubCollisions(Hub.RED_HUB, subticks);

      handleTrenchCollisions();
    }

    protected void handleHubCollisions(Hub hub, int subticks) {
      hub.handleHubInteraction(this, subticks);
      hub.fuelCollideSide(this);

      double netCollision = hub.fuelHitNet(this);
      if (netCollision != 0) {
        double px = pos.getX() + netCollision;
        pos = new Translation3d(px, pos.getY(), pos.getZ());
        vel = new Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ());
      }
    }

    protected void handleTrenchCollisions() {
      for (int i = 0; i < TRENCH_RECT_STARTS.length; i++) {
        fuelCollideRectangle(this, TRENCH_RECT_STARTS[i], TRENCH_RECT_ENDS[i]);
      }
    }

    protected void addImpulse(Translation3d impulse) {
      vel = vel.plus(impulse);
    }
  }

  protected static void handleFuelCollision(Fuel a, Fuel b) {
    double ax = a.pos.getX();
    double ay = a.pos.getY();
    double az = a.pos.getZ();
    double bx = b.pos.getX();
    double by = b.pos.getY();
    double bz = b.pos.getZ();

    double nx = ax - bx;
    double ny = ay - by;
    double nz = az - bz;

    double distanceSq = nx * nx + ny * ny + nz * nz;
    double distance;
    if (distanceSq == 0) {
      nx = 1;
      ny = 0;
      nz = 0;
      distance = 1;
    } else {
      distance = Math.sqrt(distanceSq);
      double invDistance = 1.0 / distance;
      nx *= invDistance;
      ny *= invDistance;
      nz *= invDistance;
    }

    double avx = a.vel.getX();
    double avy = a.vel.getY();
    double avz = a.vel.getZ();
    double bvx = b.vel.getX();
    double bvy = b.vel.getY();
    double bvz = b.vel.getZ();

    double relVelAlongNormal = (bvx - avx) * nx + (bvy - avy) * ny + (bvz - avz) * nz;
    double impulse = 0.5 * (1 + FUEL_COR) * relVelAlongNormal;
    double separation = (FUEL_DIAMETER - distance) * 0.5;

    a.pos = new Translation3d(ax + nx * separation, ay + ny * separation, az + nz * separation);
    b.pos = new Translation3d(bx - nx * separation, by - ny * separation, bz - nz * separation);

    a.vel = new Translation3d(avx + nx * impulse, avy + ny * impulse, avz + nz * impulse);
    b.vel = new Translation3d(bvx - nx * impulse, bvy - ny * impulse, bvz - nz * impulse);
  }

  protected static boolean fuelsOverlap(Fuel a, Fuel b) {
    double dx = a.pos.getX() - b.pos.getX();
    double dy = a.pos.getY() - b.pos.getY();
    double dz = a.pos.getZ() - b.pos.getZ();
    return dx * dx + dy * dy + dz * dz < FUEL_DIAMETER_SQ;
  }

  protected static final double CELL_SIZE = 0.25;
  protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
  protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

  @SuppressWarnings("unchecked")
  protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

  private final ArrayList<Integer> activeCellIndices = new ArrayList<>();

  protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
    // Clear grid
    for (int idx = 0; idx < activeCellIndices.size(); idx++) {
      int encoded = activeCellIndices.get(idx);
      int col = encoded / GRID_ROWS;
      int row = encoded % GRID_ROWS;
      grid[col][row].clear();
    }
    activeCellIndices.clear();

    // Populate grid
    for (Fuel fuel : fuels) {
      int col = (int) (fuel.pos.getX() / CELL_SIZE);
      int row = (int) (fuel.pos.getY() / CELL_SIZE);

      if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        ArrayList<Fuel> cell = grid[col][row];
        cell.add(fuel);
        if (cell.size() == 1) {
          activeCellIndices.add(col * GRID_ROWS + row);
        }
      }
    }

    // Check collisions only once per pair
    int activeCount = activeCellIndices.size();
    for (int idx = 0; idx < activeCount; idx++) {
      int encoded = activeCellIndices.get(idx);
      int col = encoded / GRID_ROWS;
      int row = encoded % GRID_ROWS;

      ArrayList<Fuel> cell = grid[col][row];
      int count = cell.size();

      // Within-cell pairs
      for (int a = 0; a < count; a++) {
        Fuel fuelA = cell.get(a);
        for (int b = a + 1; b < count; b++) {
          Fuel fuelB = cell.get(b);
          if (fuelsOverlap(fuelA, fuelB)) {
            handleFuelCollision(fuelA, fuelB);
          }
        }
      }

      // Neighbor cells in forward directions only
      int neighborCol = col + 1;
      if (neighborCol < GRID_COLS) {
        for (int d = -1; d <= 1; d++) {
          int neighborRow = row + d;
          if (neighborRow >= 0 && neighborRow < GRID_ROWS) {
            ArrayList<Fuel> neighborCell = grid[neighborCol][neighborRow];
            if (!neighborCell.isEmpty()) {
              for (int a = 0; a < count; a++) {
                Fuel fuelA = cell.get(a);
                for (int b = 0; b < neighborCell.size(); b++) {
                  Fuel fuelB = neighborCell.get(b);
                  if (fuelsOverlap(fuelA, fuelB)) {
                    handleFuelCollision(fuelA, fuelB);
                  }
                }
              }
            }
          }
        }
      }

      int sameColNeighborRow = row + 1;
      if (sameColNeighborRow < GRID_ROWS) {
        ArrayList<Fuel> neighborCell = grid[col][sameColNeighborRow];
        if (!neighborCell.isEmpty()) {
          for (int a = 0; a < count; a++) {
            Fuel fuelA = cell.get(a);
            for (int b = 0; b < neighborCell.size(); b++) {
              Fuel fuelB = neighborCell.get(b);
              if (fuelsOverlap(fuelA, fuelB)) {
                handleFuelCollision(fuelA, fuelB);
              }
            }
          }
        }
      }
    }
  }

  protected ArrayList<Fuel> fuels = new ArrayList<>(STARTING_FUEL_COUNT);
  protected ArrayList<Fuel> recycledFuel = new ArrayList<>(STARTING_FUEL_COUNT);
  protected boolean running = false;
  protected boolean simulateAirResistance = false;
  protected Supplier<Pose2d> robotPoseSupplier = null;
  protected Supplier<ChassisSpeeds> robotFieldSpeedsSupplier = null;
  protected double robotWidth; // size along the robot's y axis
  protected double robotLength; // size along the robot's x axis
  protected double bumperHeight;
  protected ArrayList<SimIntake> intakes = new ArrayList<>();
  protected int subticks = 1;
  protected double loggingFreqHz = 1;
  protected Timer loggingTimer = new Timer();

  /**
   * Creates a new instance of FuelSim
   * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d} structs.
   */
  public FuelSim(String tableKey) {
    // Initialize grid
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        grid[i][j] = new ArrayList<Fuel>();
      }
    }

    fuelPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(tableKey + "/Fuels", Translation3d.struct)
      .publish();
  }

  /**
   * Creates a new instance of FuelSim with log path "/Fuel Simulation"
   */
  public FuelSim() {
    this("/Fuel Simulation");
  }

  /**
   * Clears the field of fuel
   */
  public void clearFuel() {
    recycledFuel.ensureCapacity(recycledFuel.size() + fuels.size());
    recycledFuel.addAll(fuels);
    fuels.clear();
  }

  protected void recycleFuel(Fuel fuel) {
    recycledFuel.add(fuel);
  }

  protected Fuel acquireFuel(Translation3d pos, Translation3d vel) {
    int last = recycledFuel.size() - 1;
    if (last >= 0) {
      Fuel fuel = recycledFuel.remove(last);
      fuel.reset(pos, vel);
      return fuel;
    }
    return new Fuel(pos, vel);
  }

  /**
   * Spawns fuel in the neutral zone and depots
   */
  public void spawnStartingFuel() {
    fuels.ensureCapacity(fuels.size() + STARTING_FUEL_COUNT);

    Translation3d[] startingFuelPositions = STARTING_FUEL_POSITIONS;
    for (int i = 0; i < startingFuelPositions.length; i++) {
      fuels.add(acquireFuel(startingFuelPositions[i], ZERO_TRANSLATION));
    }

    // DEBUG: Log XZ lines
    // Translation3d[][] lines = new Translation3d[FIELD_XZ_LINE_STARTS.length][2];
    // for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
    //     lines[i][0] = FIELD_XZ_LINE_STARTS[i];
    //     lines[i][1] = FIELD_XZ_LINE_ENDS[i];
    // }

    // Logger.recordOutput("Fuel Simulation/Lines (debug)", lines);
  }

  protected StructArrayPublisher<Translation3d> fuelPublisher;

  /**
   * Adds array of `Translation3d`'s to NetworkTables at tableKey + "/Fuels"
   */
  public void logFuels() {
    fuelPublisher.set(fuels.stream().map((fuel) -> fuel.pos).toArray(Translation3d[]::new));
  }

  /**
   * Start the simulation. `updateSim` must still be called every loop
   */
  public void start() {
    running = true;
    loggingTimer.restart();
  }

  /**
   * Pause the simulation.
   */
  public void stop() {
    running = false;
    loggingTimer.stop();
  }

  /** Enables accounting for drag force in physics step **/
  public void enableAirResistance() {
    simulateAirResistance = true;
  }

  /**
   * Sets the number of physics iterations per loop (0.02s)
   * @param subticks physics iteration per loop (default: 5)
   */
  public void setSubticks(int subticks) {
    this.subticks = subticks;
  }

  /**
   * Sets the frequency to publish fuel translations to NetworkTables
   * Used to improve performance in AdvantageScope
   * @param loggingFreqHz update frequency in hertz
   */
  public void setLoggingFrequency(double loggingFreqHz) {
    this.loggingFreqHz = loggingFreqHz;
  }

  /**
   * Registers a robot with the fuel simulator
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight
   * @param poseSupplier
   * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
   */
  public void registerRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.robotWidth = width;
    this.robotLength = length;
    this.bumperHeight = bumperHeight;
      }

  /**
   * Registers a robot with the fuel simulator
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight from the ground
   * @param poseSupplier
   * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
   */
  public void registerRobot(
      Distance width,
      Distance length,
      Distance bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.robotPoseSupplier = poseSupplier;
    this.robotFieldSpeedsSupplier = fieldSpeedsSupplier;
    this.robotWidth = width.in(Meters);
    this.robotLength = length.in(Meters);
    this.bumperHeight = bumperHeight.in(Meters);
      }

  /**
   * To be called periodically
   * Will do nothing if sim is not running
   */
  public void updateSim() {
    if (!running) return;

    stepSim();
  }

  /**
   * Run the simulation forward 1 time step (0.02s)
   */
  public void stepSim() {

    for (int i = 0; i < subticks; i++) {
      for (Fuel fuel : fuels) {
        fuel.update(this.simulateAirResistance, this.subticks);
      }

      //handleFuelCollisions(fuels);

      if (robotPoseSupplier != null) {
        handleRobotCollisions(fuels);
        handleIntakes(fuels);
      }
    }

    if (loggingTimer.advanceIfElapsed(1.0 / loggingFreqHz)) {
      logFuels();
    }
  }

  /**
   * Adds a fuel onto the field
   * @param pos Position to spawn at
   * @param vel Initial velocity vector
   */
  public void spawnFuel(Translation3d pos, Translation3d vel) {
    fuels.add(acquireFuel(pos, vel));
  }

  /**
   * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot movement
   * @param launchVelocity Initial launch velocity
   * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching straight up
   * @param turretYaw <i>Robot-relative</i> turret yaw
   * @param launchHeight Height of the fuel to launch at. Make sure this is higher than your robot's bumper height, or else it will collide with your robot immediately.
   * @throws IllegalStateException if robot is not registered
   */
  public void launchFuel(LinearVelocity launchVelocity, Angle hoodAngle, Angle turretYaw, Distance launchHeight) {
    launchFuel(
        launchVelocity.in(MetersPerSecond),
        hoodAngle.in(Radians),
        turretYaw.in(Radians),
        launchHeight.in(Meters));
  }

  public void launchFuel(
      double launchVelocityMetersPerSecond,
      double hoodAngleRadians,
      double turretYawRadians,
      double launchHeightMeters) {
    if (robotPoseSupplier == null || robotFieldSpeedsSupplier == null) {
      throw new IllegalStateException("Robot must be registered before launching fuel.");
    }

    Pose2d robotPose = this.robotPoseSupplier.get();
    ChassisSpeeds fieldSpeeds = this.robotFieldSpeedsSupplier.get();
    double robotYaw = robotPose.getRotation().getRadians();
    double launchYaw = turretYawRadians + robotYaw;

    double horizontalVel = Math.cos(hoodAngleRadians) * launchVelocityMetersPerSecond;
    double verticalVel = Math.sin(hoodAngleRadians) * launchVelocityMetersPerSecond;
    double xVel = horizontalVel * Math.cos(launchYaw);
    double yVel = horizontalVel * Math.sin(launchYaw);

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    fuels.add(
        acquireFuel(
            new Translation3d(robotPose.getX(), robotPose.getY(), launchHeightMeters),
            new Translation3d(xVel, yVel, verticalVel)));
  }

  protected void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel) {
    Translation2d relativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
      .relativeTo(robot)
      .getTranslation();

    if (fuel.pos.getZ() > bumperHeight) return; // above bumpers
    double distanceToBottom = -FUEL_RADIUS - robotLength / 2 - relativePos.getX();
    double distanceToTop = -FUEL_RADIUS - robotLength / 2 + relativePos.getX();
    double distanceToRight = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY();
    double distanceToLeft = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY();

    // not inside robot
    if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0) return;

    Translation2d posOffset;
    // find minimum distance to side and send corresponding collision response
    if ((distanceToBottom >= distanceToTop
          && distanceToBottom >= distanceToRight
          && distanceToBottom >= distanceToLeft)) {
      posOffset = new Translation2d(distanceToBottom, 0);
    } else if ((distanceToTop >= distanceToBottom
          && distanceToTop >= distanceToRight
          && distanceToTop >= distanceToLeft)) {
      posOffset = new Translation2d(-distanceToTop, 0);
    } else if ((distanceToRight >= distanceToBottom
          && distanceToRight >= distanceToTop
          && distanceToRight >= distanceToLeft)) {
      posOffset = new Translation2d(0, distanceToRight);
    } else {
      posOffset = new Translation2d(0, -distanceToLeft);
    }

    posOffset = posOffset.rotateBy(robot.getRotation());
    fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
    Translation2d normal = posOffset.div(posOffset.getNorm());
    if (fuel.vel.toTranslation2d().dot(normal) < 0)
      fuel.addImpulse(
          new Translation3d(normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
    if (robotVel.dot(normal) > 0) fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
  }

  protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
    Pose2d robot = robotPoseSupplier.get();
    ChassisSpeeds speeds = robotFieldSpeedsSupplier.get();
    Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    for (Fuel fuel : fuels) {
      handleRobotCollision(fuel, robot, robotVel);
    }
  }

  protected void handleIntakes(ArrayList<Fuel> fuels) {
    Pose2d robot = robotPoseSupplier.get();
    for (SimIntake intake : intakes) {
      for (int i = 0; i < fuels.size(); i++) {
        Fuel fuel = fuels.get(i);
        if (intake.shouldIntake(fuel, robot)) {
          fuels.remove(i);
          recycleFuel(fuel);
          i--;
        }
      }
    }
  }

  protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
    double px = fuel.pos.getX();
    double py = fuel.pos.getY();
    double pz = fuel.pos.getZ();

    double startX = start.getX();
    double startY = start.getY();
    double startZ = start.getZ();
    double endX = end.getX();
    double endY = end.getY();
    double endZ = end.getZ();

    if (pz > endZ + FUEL_RADIUS || pz < startZ - FUEL_RADIUS) return;

    double distanceToLeft = startX - FUEL_RADIUS - px;
    double distanceToRight = px - endX - FUEL_RADIUS;
    double distanceToTop = py - endY - FUEL_RADIUS;
    double distanceToBottom = startY - FUEL_RADIUS - py;

    if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) return;

    double offsetX;
    double offsetY;
    if (px < startX
        || (distanceToLeft >= distanceToRight
          && distanceToLeft >= distanceToTop
          && distanceToLeft >= distanceToBottom)) {
      offsetX = distanceToLeft;
      offsetY = 0;
    } else if (px >= endX
        || (distanceToRight >= distanceToLeft
          && distanceToRight >= distanceToTop
          && distanceToRight >= distanceToBottom)) {
      offsetX = -distanceToRight;
      offsetY = 0;
    } else if (py > endY
        || (distanceToTop >= distanceToLeft
          && distanceToTop >= distanceToRight
          && distanceToTop >= distanceToBottom)) {
      offsetX = 0;
      offsetY = -distanceToTop;
    } else {
      offsetX = 0;
      offsetY = distanceToBottom;
    }

    if (offsetX != 0) {
      fuel.pos = new Translation3d(px + offsetX, py, pz);
      fuel.vel = new Translation3d(-(FIELD_COR) * fuel.vel.getX(), fuel.vel.getY(), fuel.vel.getZ());
    } else if (offsetY != 0) {
      fuel.pos = new Translation3d(px, py + offsetY, pz);
      fuel.vel = new Translation3d(fuel.vel.getX(), -(FIELD_COR) * fuel.vel.getY(), fuel.vel.getZ());
    }
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
    intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
      }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   */
  public void registerIntake(double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
    registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   */
  public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
    registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
  }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(
      Distance xMin,
      Distance xMax,
      Distance yMin,
      Distance yMax,
      BooleanSupplier ableToIntake,
      Runnable intakeCallback) {
    registerIntake(
        xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake, intakeCallback);
      }

  /**
   * Registers an intake with the fuel simulator. This intake will remove fuel from the field based on the `ableToIntake` parameter.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param ableToIntake Should a return a boolean whether the intake is active
   */
  public void registerIntake(
      Distance xMin, Distance xMax, Distance yMin, Distance yMax, BooleanSupplier ableToIntake) {
    registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), ableToIntake);
      }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   * @param intakeCallback Function to call when a fuel is intaked
   */
  public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax, Runnable intakeCallback) {
    registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters), intakeCallback);
  }

  /**
   * Registers an intake with the fuel simulator. This intake will always remove fuel from the field.
   * @param xMin Minimum x position for the bounding box
   * @param xMax Maximum x position for the bounding box
   * @param yMin Minimum y position for the bounding box
   * @param yMax Maximum y position for the bounding box
   */
  public void registerIntake(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
    registerIntake(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
  }

  public static class Hub {
    public static final Hub BLUE_HUB =
      new Hub(new Translation2d(4.61, FIELD_WIDTH / 2), new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
    public static final Hub RED_HUB = new Hub(
        new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
        new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
        -1);

    protected static final double ENTRY_HEIGHT = 1.83;
    protected static final double ENTRY_RADIUS = 0.56;
    protected static final double ENTRY_RADIUS_SQ = ENTRY_RADIUS * ENTRY_RADIUS;

    protected static final double SIDE = 1.2;

    protected static final double NET_HEIGHT_MAX = 3.057;
    protected static final double NET_HEIGHT_MIN = 1.5;
    protected static final double NET_OFFSET = SIDE / 2 + 0.261;
    protected static final double NET_WIDTH = 1.484;

    protected final Translation2d center;
    protected final double centerX;
    protected final double centerY;
    protected final Translation3d exit;
    protected final int exitVelXMult;
    protected final Translation3d sideStart;
    protected final Translation3d sideEnd;

    protected int score = 0;

    protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
      this.center = center;
      this.centerX = center.getX();
      this.centerY = center.getY();
      this.exit = exit;
      this.exitVelXMult = exitVelXMult;
      this.sideStart = new Translation3d(centerX - SIDE / 2, centerY - SIDE / 2, 0);
      this.sideEnd = new Translation3d(centerX + SIDE / 2, centerY + SIDE / 2, ENTRY_HEIGHT - 0.1);
    }

    protected void handleHubInteraction(Fuel fuel, int subticks) {
      if (didFuelScore(fuel, subticks)) {
        fuel.pos = exit;
        fuel.vel = getDispersalVelocity();
        score++;
      }
    }

    protected boolean didFuelScore(Fuel fuel, int subticks) {
      double dx = fuel.pos.getX() - centerX;
      double dy = fuel.pos.getY() - centerY;
      return dx * dx + dy * dy <= ENTRY_RADIUS_SQ
        && fuel.pos.getZ() <= ENTRY_HEIGHT
        && fuel.pos.getZ() - fuel.vel.getZ() * (PERIOD / subticks) > ENTRY_HEIGHT;
    }

    protected Translation3d getDispersalVelocity() {
      return new Translation3d(exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
    }

    /**
     * Reset this hub's score to 0
     */
    public void resetScore() {
      score = 0;
    }

    /**
     * Get the current count of fuel scored in this hub
     * @return
     */
    public int getScore() {
      return score;
    }

    protected void fuelCollideSide(Fuel fuel) {
      fuelCollideRectangle(fuel, sideStart, sideEnd);
    }

    protected double fuelHitNet(Fuel fuel) {
      if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) return 0;
      if (fuel.pos.getY() > centerY + NET_WIDTH / 2 || fuel.pos.getY() < centerY - NET_WIDTH / 2)
        return 0;
      double netX = centerX + NET_OFFSET * exitVelXMult;
      if (fuel.pos.getX() > netX) {
        return Math.max(0, netX - (fuel.pos.getX() - FUEL_RADIUS));
      } else {
        return Math.min(0, netX - (fuel.pos.getX() + FUEL_RADIUS));
      }
    }
  }

  protected class SimIntake {
    double xMin, xMax, yMin, yMax;
    BooleanSupplier ableToIntake;
    Runnable callback;

    protected SimIntake(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier ableToIntake,
        Runnable intakeCallback) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.ableToIntake = ableToIntake;
      this.callback = intakeCallback;
        }

    protected boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
      if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) return false;

      Translation2d fuelRelativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
        .relativeTo(robotPose)
        .getTranslation();

      boolean result = fuelRelativePos.getX() >= xMin
        && fuelRelativePos.getX() <= xMax
        && fuelRelativePos.getY() >= yMin
        && fuelRelativePos.getY() <= yMax;
      if (result) {
        callback.run();
      }
      return result;
    }
  }
}
