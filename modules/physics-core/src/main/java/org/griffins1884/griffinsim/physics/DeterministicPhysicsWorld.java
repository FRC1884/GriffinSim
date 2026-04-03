package org.griffins1884.griffinsim.physics;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.griffins1884.griffinsim.contracts.RigidBodyState;

public final class DeterministicPhysicsWorld {
  public ImmutableWorldState step(ImmutableWorldState current, PhysicsStepRequest request) {
    return stepDetailed(current, request).worldState();
  }

  public PhysicsStepResult stepDetailed(ImmutableWorldState current, PhysicsStepRequest request) {
    Map<String, BodyCommand> commandsByBody =
        request.commands().stream()
            .sorted(Comparator.comparing(BodyCommand::bodyId))
            .collect(Collectors.toMap(BodyCommand::bodyId, Function.identity(), (left, right) -> right));
    Map<String, List<ContactConstraint>> contactsByBody =
        DeterministicContactOrdering.sort(request.contacts()).stream()
            .collect(Collectors.groupingBy(ContactConstraint::bodyId, Collectors.toList()));

    double dtSeconds = request.stepNanos() / 1_000_000_000.0;
    List<ContactEvent> allEvents = new ArrayList<>();
    List<RigidBodyState> nextBodies =
        CanonicalBodyOrdering.sort(current.bodies()).stream()
            .map(body -> stepBody(body, commandsByBody.get(body.bodyId()), contactsByBody.get(body.bodyId()), request.gravityMps2(), dtSeconds, allEvents))
            .toList();
    return new PhysicsStepResult(new ImmutableWorldState(request.nextHeader(), nextBodies), allEvents);
  }

  private static RigidBodyState stepBody(
      RigidBodyState body,
      BodyCommand command,
      List<ContactConstraint> contacts,
      double gravityMps2,
      double dtSeconds,
      List<ContactEvent> allEvents) {
    if (command == null) {
      return body;
    }

    double ax = command.forceX() / command.massKg();
    double ay = command.forceY() / command.massKg();
    double az = (command.forceZ() / command.massKg()) - gravityMps2;
    double wx = body.wx() + (command.torqueX() / command.momentOfInertiaKgM2()) * dtSeconds;
    double wy = body.wy() + (command.torqueY() / command.momentOfInertiaKgM2()) * dtSeconds;
    double wz = body.wz() + (command.torqueZ() / command.momentOfInertiaKgM2()) * dtSeconds;
    double vx = body.vx() + ax * dtSeconds;
    double vy = body.vy() + ay * dtSeconds;
    double vz = body.vz() + az * dtSeconds;
    double x = body.x() + vx * dtSeconds;
    double y = body.y() + vy * dtSeconds;
    double z = body.z() + vz * dtSeconds;

    if (contacts != null) {
      for (ContactConstraint contact : contacts) {
        if (contact.penetrationMeters() <= 0.0) {
          continue;
        }
        x += contact.normalX() * contact.penetrationMeters();
        y += contact.normalY() * contact.penetrationMeters();
        z += contact.normalZ() * contact.penetrationMeters();

        double normalSpeed = (vx * contact.normalX()) + (vy * contact.normalY()) + (vz * contact.normalZ());
        double tangentX = vx - (normalSpeed * contact.normalX());
        double tangentY = vy - (normalSpeed * contact.normalY());
        double tangentZ = vz - (normalSpeed * contact.normalZ());
        double tangentialSpeed = Math.sqrt((tangentX * tangentX) + (tangentY * tangentY) + (tangentZ * tangentZ));

        if (normalSpeed < 0.0) {
          double postNormalSpeed = -normalSpeed * contact.restitutionCoefficient();
          double deltaNormal = postNormalSpeed - normalSpeed;
          vx += deltaNormal * contact.normalX();
          vy += deltaNormal * contact.normalY();
          vz += deltaNormal * contact.normalZ();
        }

        double frictionScale = Math.max(0.0, 1.0 - Math.min(1.0, contact.frictionCoefficient()));
        double updatedNormalSpeed = (vx * contact.normalX()) + (vy * contact.normalY()) + (vz * contact.normalZ());
        double updatedTangentX = vx - (updatedNormalSpeed * contact.normalX());
        double updatedTangentY = vy - (updatedNormalSpeed * contact.normalY());
        double updatedTangentZ = vz - (updatedNormalSpeed * contact.normalZ());
        vx = (updatedNormalSpeed * contact.normalX()) + (updatedTangentX * frictionScale);
        vy = (updatedNormalSpeed * contact.normalY()) + (updatedTangentY * frictionScale);
        vz = (updatedNormalSpeed * contact.normalZ()) + (updatedTangentZ * frictionScale);

        double angularNormal = (wx * contact.normalX()) + (wy * contact.normalY()) + (wz * contact.normalZ());
        double angularTangentX = wx - (angularNormal * contact.normalX());
        double angularTangentY = wy - (angularNormal * contact.normalY());
        double angularTangentZ = wz - (angularNormal * contact.normalZ());
        double rollingScale = Math.max(0.0, 1.0 - Math.min(1.0, contact.rollingFrictionCoefficient()));
        double torsionalScale = Math.max(0.0, 1.0 - Math.min(1.0, contact.torsionalFrictionCoefficient()));
        wx = (angularNormal * torsionalScale * contact.normalX()) + (angularTangentX * rollingScale);
        wy = (angularNormal * torsionalScale * contact.normalY()) + (angularTangentY * rollingScale);
        wz = (angularNormal * torsionalScale * contact.normalZ()) + (angularTangentZ * rollingScale);

        double tangentialSpeedAfter = Math.sqrt((updatedTangentX * updatedTangentX * frictionScale * frictionScale)
            + (updatedTangentY * updatedTangentY * frictionScale * frictionScale)
            + (updatedTangentZ * updatedTangentZ * frictionScale * frictionScale));
        double slipRatio = tangentialSpeed <= 1e-9 ? 0.0 : Math.max(0.0, Math.min(1.0, (tangentialSpeed - tangentialSpeedAfter) / tangentialSpeed));
        allEvents.add(new ContactEvent(body.bodyId(), contact.contactId(), contact.penetrationMeters(), normalSpeed, tangentialSpeed, tangentialSpeedAfter, slipRatio, contact.frictionCoefficient(), contact.restitutionCoefficient(), contact.rollingFrictionCoefficient(), contact.torsionalFrictionCoefficient()));
      }
    }

    QuaternionState nextOrientation = integrateQuaternion(body, wx, wy, wz, dtSeconds);
    return new RigidBodyState(
        body.bodyId(),
        x,
        y,
        z,
        nextOrientation.qw(),
        nextOrientation.qx(),
        nextOrientation.qy(),
        nextOrientation.qz(),
        vx,
        vy,
        vz,
        wx,
        wy,
        wz);
  }

  private static QuaternionState integrateQuaternion(RigidBodyState body, double wx, double wy, double wz, double dtSeconds) {
    double halfDt = 0.5 * dtSeconds;
    double dqW = (-body.qx() * wx - body.qy() * wy - body.qz() * wz) * halfDt;
    double dqX = (body.qw() * wx + body.qy() * wz - body.qz() * wy) * halfDt;
    double dqY = (body.qw() * wy + body.qz() * wx - body.qx() * wz) * halfDt;
    double dqZ = (body.qw() * wz + body.qx() * wy - body.qy() * wx) * halfDt;
    double qw = body.qw() + dqW;
    double qx = body.qx() + dqX;
    double qy = body.qy() + dqY;
    double qz = body.qz() + dqZ;
    double norm = Math.sqrt((qw * qw) + (qx * qx) + (qy * qy) + (qz * qz));
    if (norm <= 1e-12) {
      return new QuaternionState(1.0, 0.0, 0.0, 0.0);
    }
    return new QuaternionState(qw / norm, qx / norm, qy / norm, qz / norm);
  }

  private record QuaternionState(double qw, double qx, double qy, double qz) {}
}
