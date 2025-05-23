# PhysECS

Physecs is a real-time 3D rigid body physics engine using a sequential impulses TGS solver and built on top of [EnTT](https://github.com/skypjack/entt) ECS.

![ezgif-6eaa905df3ca2c](https://github.com/user-attachments/assets/413fecb5-1cd7-4276-a38b-a629f280085f)

## Usage

### Components

Physecs is intended to share the same transform component as your application, defined in [Transform.h](src/Transform.h). 
It must be the same as the transform component in the main application, and include the fields `glm::vec3 position` and `glm::quat orientation`.

```c++
struct TransformComponent {
    glm::vec3 position;
    glm::quat orientation;
    glm::vec3 scale;
};
```

Additionally Physecs defines two components `RigidBodyCollisionComponent` and `RigidBodyDynamicComponent`. 

```c++
struct RigidBodyCollisionComponent {
    std::vector<Collider> colliders;
};

struct RigidBodyDynamicComponent {
    float invMass;
    glm::vec3 com;
    glm::vec3 velocity;
    glm::mat3 invInertiaTensor;
    glm::vec3 angularVelocity;
    bool isKinematic;
    glm::mat3 invInertiaTensorWorld;
};
```

`RigidBodyCollisionComponent` defines the shapes the entity uses for collision, and `RigidBodyDynamicComponent` defines the properties of the entity used for simulating motion. If an entity has a `RigidBodyCollisionComponent` but not a `RigidBodyDynamicComponent` it is treated as a static body.

Currently the supported shapes are:
- Sphere
- Capsule
- Box
- Convex mesh
- Triangle mesh (static only)

### Joints

Physecs also supports joints.
The following joints are included:
- Fixed
- Revolute
- Spherical
- Gear
- Prismatic
- Servo
- Universal (U-joint)

But new joints can be added by subclassing the `Joint` class.

### Simulation

A Physecs scene takes an `entt::registry` in it's constructor. 
```c++
physecs::Scene physicsScene = physecs::Scene(registry);
```
In order to advance the simulation by an amount of time, call the `void simulate(float timeStep)` function on it.
You can change the number of substeps and iterations on the physics scene to create more or less fidelity.

Check out the demo for an example of usage.

## Dependencies

- EnTT 3.12.2
- GLM 0.9.9.8
- OpenGL 3.3 (Demo)
- SDL2 (Demo)
- TinyGltf (Demo)
- STB image (Demo)
- C++ standard library

## Requirements

C++ 17

## Why build physics on top of EnTT and GLM?

I am developing this physics library for a specific game I am making and my engine uses EnTT and GLM.

## Future plans

- Fix bodies tilting when bouncing off a flat surface when there is multiple contact points
- Add multithreading
- Continuous collision detection

## References

These are the some of the materials I used to learn about physics simulation and collision detection and other helpful resources:

- https://box2d.org/publications/
- https://box2d.org/posts/2024/02/solver2d/
- Christer Ericson: "Real-Time Collision Detection"
- Philip J. Schneider, David H. Eberly: "Geometric Tools for Computer Graphics"
- David H. Eberly: "3D Game Engine Design: A Practical Approach to Real-Time Computer Graphics"
- https://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
- https://allenchou.net/game-physics-series/
- https://github.com/kevinmoran/GJK
- https://www.codercorner.com/MeshContacts.pdf








