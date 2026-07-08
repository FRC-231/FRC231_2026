# FRC 231 — 2026 Robot Code Guide

Welcome! This document explains how our robot's code works in plain language. If
you know a little bit of programming and you've seen the robot drive around,
this guide should help you understand *why* it does what it does and *where* to
look in the code when you want to change something.

The robot is programmed in **C++** using **WPILib** (the standard FRC robot
library). The most important idea to understand up front is this:

> The robot is split into **subsystems** (Drive, Intake, Shooter, Climb). Each
> subsystem is a self-contained chunk of code that controls one part of the
> robot. A central "brain" file (`Robot.cpp`) reads the controllers and tells
> each subsystem what to do.

---

## Table of Contents

1. [The Big Picture: How a Robot Program Runs](#1-the-big-picture-how-a-robot-program-runs)
2. [Project Layout: Where Everything Lives](#2-project-layout-where-everything-lives)
3. [Core Concepts You'll See Everywhere](#3-core-concepts-youll-see-everywhere)
4. [C++ and Programming Building Blocks](#4-c-and-programming-building-blocks)
5. [The Robot Class (the "Brain")](#5-the-robot-class-the-brain)
6. [RobotContext (Shared Hardware)](#6-robotcontext-shared-hardware)
7. [Subsystem: Drive](#7-subsystem-drive)
8. [Subsystem: Intake](#8-subsystem-intake)
9. [Subsystem: Shooter](#9-subsystem-shooter)
10. [Subsystem: Climb](#10-subsystem-climb)
11. [Vision (Cameras and the AprilTags)](#11-vision-cameras-and-the-apriltags)
12. [The Lua Scripting System (Autonomous)](#12-the-lua-scripting-system-autonomous)
13. [The Config System (Tuning Without Recompiling)](#13-the-config-system-tuning-without-recompiling)
14. [Network Tables (Talking to the Laptop)](#14-network-tables-talking-to-the-laptop)
15. [The HighVoltage Library (Our Reusable Toolbox)](#15-the-highvoltage-library-our-reusable-toolbox)
16. [Glossary](#16-glossary)

---

## 1. The Big Picture: How a Robot Program Runs

An FRC robot doesn't run like a normal program that starts at the top and
finishes at the bottom. Instead, WPILib calls certain functions **over and over,
50 times every second** (once every 20 milliseconds). This repeating cycle is
called the **loop** or a **"periodic"** function.

Our main class, `Robot`, inherits from `frc::TimedRobot`. WPILib decides what
"mode" the robot is in and calls the matching functions:

| Mode | When it happens | Functions called |
|------|-----------------|------------------|
| **Disabled** | Robot is on but not allowed to move | `DisabledInit`, `DisabledPeriodic` |
| **Autonomous** | First 15 seconds of a match, robot drives itself | `AutonomousInit`, `AutonomousPeriodic` |
| **Teleop** | Drivers control the robot with controllers | `TeleopInit`, `TeleopPeriodic` |
| **Test** | For testing/tuning in the pit | `TestInit`, `TestPeriodic` |

The `*Init` function runs **once** when that mode starts. The `*Periodic`
function runs **50 times per second** the whole time you're in that mode.

There is also `RobotPeriodic()`, which runs every loop **no matter what mode**
you're in. This is where the subsystems get their regular "heartbeat."

You can find all of this in [src/main/cpp/Robot.cpp](src/main/cpp/Robot.cpp).

---

## 2. Project Layout: Where Everything Lives

```
src/
├── main/
│   ├── include/        ← Header files (.h) — declare what code exists
│   ├── cpp/            ← Source files (.cpp) — the actual code
│   └── deploy/         ← Files copied onto the robot (configs + Lua scripts)
│       ├── config/     ← JSON tuning files (BaseRobot.json, TestRobot.json)
│       └── scripts/
│           └── autons/ ← Lua autonomous routines
│
├── HighVoltageRobotLib/  ← Our reusable robot toolbox (swerve, PID, utilities)
├── HighVoltageLuaLib/    ← The glue that lets Lua scripts call C++ code
└── test/                 ← Unit tests
```

**Header (`.h`) vs Source (`.cpp`):** In C++, a header file is like a table of
contents — it lists the functions and variables that exist. The `.cpp` file
contains the actual instructions for those functions. When you want to know
*what a subsystem can do*, read the `.h`. When you want to know *how it does
it*, read the `.cpp`.

---

## 3. Core Concepts You'll See Everywhere

Before diving into subsystems, here are patterns that repeat throughout the code.

### Subsystems

Every major mechanism is a class that inherits from `frc2::SubsystemBase`. Each
one has a `Periodic()` function that WPILib runs automatically 50 times a
second. The subsystem keeps track of what it *should* be doing (its "mode") and
`Periodic()` makes the motors actually do it.

### The "Mode + State Machine" Pattern

This is the single most important pattern in our code. Instead of immediately
spinning a motor when a button is pressed, we just **record what we want** by
setting a "mode" variable. Then, every loop, `Periodic()` looks at the current
mode and acts on it.

For example, in the Intake:

```cpp
void IntakeSystem::feedIntake()   // Called when a button is pressed
{
    feed_mode = FeedMode::INTAKE;  // Just remember "we want to intake"
}

void IntakeSystem::Periodic()      // Runs 50x/sec
{
    switch(feed_mode)              // Look at what we want, and do it
    {
        case FeedMode::INTAKE: doFeedIntake(); break;
        case FeedMode::STOP:   doFeedStop();   break;
        // ...
    }
}
```

Why do it this way? It keeps things predictable. The motor command is sent in
exactly one place, at a steady rate, no matter how many times or how fast a
button gets pressed.

### Units

WPILib uses **typed units** so you can't accidentally mix up, say, meters and
degrees. You'll see things like `5.0_mps` (meters per second), `90_deg`
(degrees), `7_V` (volts), and `4500_rpm`. To get a plain number out, you call
`.value()`. This catches a whole category of bugs at compile time.

### PID Controllers

A **PID controller** is a piece of math that smoothly drives an error toward
zero. You give it a *target* (e.g. "point the wheel at 90°") and the *current
value*, and it calculates how much power to apply. It's used for aiming,
positioning the intake pivot, holding the shooter at a target speed, and more.
You don't need to understand the math to use it — just know that `kP`, `kI`,
and `kD` are the three knobs that tune how aggressively it corrects.

### Feed Forward

A **feed forward** is a "best guess" of how much power something needs *before*
the PID even kicks in. For example, to spin a wheel at a known speed, physics
tells us roughly what voltage to use. Feed forward supplies that base amount and
PID just fixes the small leftover error. The knobs are `kS` (static friction),
`kV` (velocity), and `kA` (acceleration).

---

## 4. C++ and Programming Building Blocks

This section explains the general programming ideas the rest of this guide
assumes you know. If you've only written a little code before, read this once and
the subsystem chapters will make a lot more sense. None of this is specific to
robots — it's how C++ programs are built in general.

### Classes and Objects (the heart of OOP)

C++ is an **object-oriented** language. "Object-oriented programming" (OOP) is a
style where you bundle **data** (variables) and the **functions that work on that
data** together into one unit called a **class**.

Think of a class as a *blueprint* and an object as a *thing built from that
blueprint*. The `ShooterSystem` class is the blueprint that describes what a
shooter has (motors, target speeds) and what it can do (`autoShoot`,
`isAtSpeed`). When the robot boots, it builds one actual `ShooterSystem`
**object** from that blueprint:

```cpp
shooter_system = std::make_shared<ShooterSystem>(context);
//   ^ the object              ^ the class (blueprint)
```

A class has:
- **Member variables** — the data each object stores (e.g. `left_shooter_motor`,
  `shooter_mode`).
- **Member functions (methods)** — actions the object can perform. You call them
  with a dot: `shooter_system->autoShoot()`. (We use `->` instead of `.` here
  because `shooter_system` is a pointer — more on that below.)

### Public vs. Private (encapsulation)

Inside a class you'll see the words `public:` and `private:`. This is
**encapsulation** — controlling what the outside world is allowed to touch.

- **`public`** members can be used by anyone. These are the "buttons" other code
  is allowed to press, like `autoShoot()`.
- **`private`** members can only be used by the class itself. These are the
  internal guts, like the individual motor objects and helper functions
  (`doShooterAutoControl()`).

Why hide things? So the rest of the code can only interact with a subsystem in
safe, intended ways. You can't accidentally set a motor voltage from across the
codebase — you have to go through a public function that does it correctly. This
is why every subsystem exposes simple commands like `feedIntake()` and keeps the
messy motor details private.

### The Constructor

Every class has a special function called the **constructor** that runs once,
automatically, when an object is created. It has the same name as the class. Its
job is to set up the object — read the config, create the motors, configure them.

```cpp
ShooterSystem::ShooterSystem(std::shared_ptr<RobotContext> context):
    context(context)        // ← this part stores the passed-in context
{
    // ... create motors, read config values, etc. ...
}
```

In our subsystems, the constructor is where all the one-time hardware setup
happens. After it finishes, `Periodic()` takes over for the repeating work.

### Inheritance and Virtual Functions

**Inheritance** lets one class build on another. The child class gets everything
the parent has, and can add to it or change parts of it. You'll see this written
with a colon:

```cpp
class ShooterSystem : public frc2::SubsystemBase
//    ^ child                 ^ parent
```

This says "a `ShooterSystem` *is a* `SubsystemBase`." Because WPILib's
`SubsystemBase` already knows how to register itself and get its `Periodic()`
called automatically, our shooter inherits all of that for free.

A **virtual function** is a function the parent *declares* but expects children
to *replace* (override). `Periodic()` is virtual in `SubsystemBase`; each
subsystem provides its own version with the `override` keyword. A **pure
virtual** function (marked `= 0`) has *no* body in the parent at all — children
are *required* to write one. That's exactly how `SwerveModule` works (see
[Section 15](#15-the-highvoltage-library-our-reusable-toolbox)): it's an abstract
blueprint, and `MK5NSwerveModule` fills in the required functions.

The payoff is **polymorphism**: code can hold a parent-type pointer
(`SwerveModule*`) and call `update()` on it without knowing or caring which
specific kind of module it really is. The right child's version runs
automatically.

### Pointers, References, and Smart Pointers

A **pointer** is a variable that doesn't hold a value directly — it holds the
*address* of where a value lives in memory. It's like a contact in your phone:
the contact isn't the person, it's a way to *reach* the person.

You'll see three flavors in our code:

- **Raw pointer (`T*`)** — the basic kind. You reach the thing it points to with
  `->` instead of `.`.
- **Reference (`T&`)** — an alias for an existing object. Often used for function
  parameters so the function works on the *real* object instead of a copy (e.g.
  `addToLuaState(Keeko::ThreadedScript& script)`).
- **Smart pointers (`std::shared_ptr<T>`, `std::unique_ptr<T>`)** — modern C++
  pointers that **automatically clean up memory** when they're no longer used, so
  you can't forget to free it. We use these almost everywhere:
  - `std::unique_ptr` — exactly *one* owner. When it goes away, the object is
    deleted. Used for motors a subsystem solely owns.
  - `std::shared_ptr` — *shared* ownership; the object stays alive as long as
    anyone holds a pointer to it. The `RobotContext` is a `shared_ptr` because
    every subsystem needs to hold onto the same one.

You don't need to master memory management to read the code — just know that
`->` means "follow this pointer to the object and use it."

### Callbacks and Lambdas

A **callback** is a function you hand to *another* piece of code so it can call
*you back* later, when something happens. Instead of constantly asking "did it
happen yet? did it happen yet?", you say "here's what to do when it happens" and
walk away.

In C++ the most common way to write a callback on the spot is a **lambda** — a
small, unnamed function written right where you need it. The syntax looks
unusual at first:

```cpp
[capture](parameters) { body }
```

- The **`[ ]`** is the *capture list* — it lists variables from the surrounding
  code that the lambda is allowed to use. `[this]` is extremely common in our
  code: it lets the lambda reach back into the object that created it, so the
  lambda can call the object's own functions and variables.
- The **`( )`** holds parameters, just like a normal function.
- The **`{ }`** is the code that runs when the lambda is called.

Here's a real example from the drive system. We tell a Network Table value "run
this code whenever someone changes the auto-aim offset from the dashboard":

```cpp
nt_auto_aim_offset.setOnUpdate([this](double offset)
{
    std::cout << "Auto aim offset update received: " << offset << std::endl;
    this->auto_aim_offset = units::degree_t{offset};   // [this] lets us reach the member
});
```

We never call that lambda ourselves. The Network Tables system holds onto it and
calls it *back* for us the moment the value changes. That's the whole idea of a
callback.

Callbacks show up all over our code:
- **Live tuning** — every `setOnUpdate(...)` registers a callback that re-applies
  a value when the dashboard changes it.
- **Autonomous waiting** — `control:waitUntilTrue(function ... end)` in Lua hands
  a callback to the C++ side, which keeps calling it until it returns true.
- **Failure handling** — `auton_script->setOnFailCallback(...)` registers a
  lambda that stops the drivetrain if a script crashes.

You'll also see lambdas stored in a type called `std::function<...>` — that's
just "a variable that can hold any callback with a matching shape."

### Enums (named choices)

An **enum** (enumeration) is a type that can only be one of a fixed set of named
values. Instead of remembering that "mode 0 means stop, mode 1 means intake," you
give the choices real names:

```cpp
enum class FeedMode { STOP, INTAKE, OUTTAKE, HALF_SPEED };
```

Now `feed_mode = FeedMode::INTAKE;` is clear and the compiler won't let you set
it to something invalid. Enums are what make the [state-machine
pattern](#the-mode--state-machine-pattern) readable — every subsystem's `switch`
statement branches on an enum.

### Templates (write once, works for many types)

A **template** is a function or class with a "blank" type that the compiler fills
in for you. It lets you write logic *once* and reuse it for many data types
instead of copy-pasting. `NetworkTableValue<double>` and
`NetworkTableValue<bool>` come from the same template with the blank filled in
differently. Templates are explained in depth in
[Section 15](#networktablevaluet--one-template-for-many-types) — for now just
know that the `<...>` after a type name is filling in a template's blank.

### Namespaces

A **namespace** is a labeled box that groups related names together so they don't
collide. Our library code lives in `namespace HighVoltage`, so a class there is
fully named `HighVoltage::MK5NSwerveModule`. WPILib uses `frc::`, and the CTRE
motor library uses `ctre::phoenix6::`. The `::` just means "look inside this
box." It's why you see those prefixes everywhere.

### Header Files vs. Source Files (a reminder)

As mentioned in [Section 2](#2-project-layout-where-everything-lives): the `.h`
**header** declares *what exists* (the class, its function names, its variables),
and the `.cpp` **source** contains the actual code for those functions. Other
files `#include` the header to learn what they're allowed to call. When you want
the "menu" of what a class can do, read the header; when you want the recipe,
read the source.

---

## 5. The Robot Class (the "Brain")

**Files:** [src/main/include/Robot.h](src/main/include/Robot.h),
[src/main/cpp/Robot.cpp](src/main/cpp/Robot.cpp)

`Robot` is the top-level class. It owns one of each subsystem and the
`RobotContext`. Here's what it does in each phase:

### Construction (boot up)

When the robot turns on, the `Robot()` constructor:
1. Loads the config file (`BaseRobot.json`) into the `RobotContext`. If the file
   fails to load, the robot throws an error and refuses to start — this is on
   purpose, because running with no config would be dangerous.
2. Creates the Drive, Intake, and Shooter subsystems.
3. Hooks the shooter up to the drive system so the shooter always knows the
   distance to the hub (this is how it picks the right shooting speed).
4. Sets up the autonomous selector (the dropdown the drivers pick from).

### Teleop (driver control)

`TeleopPeriodic()` is the heart of driver control. Every loop it:
1. Reads the pilot and copilot controllers (joysticks, buttons, triggers).
2. Translates those inputs into subsystem commands.

A few highlights of what the controls do:
- **Left stick** = drive translation, **Right stick X** = rotation.
- **L1** toggles "auto-aim," where the robot automatically rotates to face the
  hub while you drive.
- **Triangle** = half speed (for precise movements).
- **L2 trigger** = feed a ball into the shooter.
- **R1** toggles the intake pivot up/down.
- **R2 trigger** = run the intake.
- The controller **rumbles** to give the driver feedback — one side buzzes when
  the shooter is up to speed, the other buzzes when the robot reaches the
  "sweet spot" shooting distance from the hub.

### The "Sweet Spot" Rumble

There's a nice example of driver feedback in `TeleopPeriodic`. The robot
constantly computes how far it is from the ideal shooting distance
(`SWEET_STOP_DISTANCE`, 2.25 m from the hub). As you get close, the controller
starts buzzing softly, and buzzes at full strength when you're right on the
sweet spot. This helps the driver "feel" the best place to shoot from.

### Autonomous

Our autonomous routines aren't written in C++ — they're written in **Lua**
scripts (see [Section 12](#12-the-lua-scripting-system-autonomous)).
`AutonomousInit()` simply tells the already-loaded Lua script to run its
`autonomous` function. The heavy lifting of *loading* the script actually
happens earlier, during Disabled mode, so there's no delay when the match
starts.

---

## 6. RobotContext (Shared Hardware)

**Files:** [src/main/include/RobotContext.h](src/main/include/RobotContext.h),
[src/main/cpp/RobotContext.cpp](src/main/cpp/RobotContext.cpp)

Some things need to be shared by *all* subsystems: the controllers, the gyro
(Pigeon), the cameras (Vision), and the loaded config. Rather than passing all
of those around separately, we bundle them into one object called
`RobotContext`.

Every subsystem receives a `shared_ptr<RobotContext>` when it's created, so they
can all reach the same controllers, gyro, and config.

Important things `RobotContext` provides:
- **`getPilotController()` / `getCopilotController()`** — the two PS5
  controllers.
- **`getPigeon()`** — the Pigeon 2 gyro, which tells us the robot's heading
  (which way it's facing).
- **`getVision()`** — the camera system.
- **`getConfig()`** — the tuning values loaded from JSON.
- **`HUB_POSITION`** — a fixed point on the field representing the hub. Lots of
  aiming math is done relative to this point.
- **Competition mode / NT throttle** — in competition mode, the robot sends less
  data to the laptop to save bandwidth (more on this in
  [Section 14](#14-network-tables-talking-to-the-laptop)).

---

## 7. Subsystem: Drive

**Files:** [src/main/include/DriveSystem.h](src/main/include/DriveSystem.h),
[src/main/cpp/DriveSystem.cpp](src/main/cpp/DriveSystem.cpp)

This is the biggest and most complex subsystem. It controls how the robot moves
around the field.

### Swerve Drive Basics

Our robot uses **swerve drive**. Each of the four corners has a module with two
motors:
- A **drive motor** that spins the wheel (makes the robot move).
- A **rotate motor** that points the wheel in any direction.

Because each wheel can point independently, a swerve robot can move in any
direction *and* spin at the same time. Each corner module is a
`MK5NSwerveModule` object, and a `SwerveModuleCordinator` figures out the right
angle and speed for all four wheels given a desired motion.

### Field-Oriented Driving

"Field-oriented" means when the driver pushes the stick forward, the robot moves
*away from the driver station* — no matter which way the robot is actually
facing. The code makes this work by combining the stick direction with the
robot's heading from the gyro. This is much easier to drive than "robot
relative," where forward depends on which way the robot's front is pointing.

### Drive Modes (another state machine)

Just like the intake, the drive system has a `drive_mode` and a big `switch` in
`Periodic()`. The modes include:

| Mode | What it does |
|------|--------------|
| `STOP` | Don't move |
| `MANUAL` | Normal driver control |
| `MANUAL_AA` | Driver controls translation, robot auto-aims at the hub |
| `MANUAL_FACE_BACK` | Driver controls translation, robot faces backward (for "snow blower" launching) |
| `DRIVE_TO` | Drive to a specific (x, y, angle) point — used in autonomous |
| `ROTATE_TO` | Rotate to a specific heading |
| `FOLLOW_TRAJECTORY` | Follow a smooth, curved path |
| `DRIVE_FF` / `TURN_FF` | Special modes for measuring motor characteristics during tuning |

### Odometry (knowing where you are)

**Odometry** is how the robot tracks its own position on the field. It combines:
1. How far each wheel has rolled and which way it pointed.
2. The gyro heading.
3. Camera data (from AprilTags — see [Section 11](#11-vision-cameras-and-the-apriltags)).

These are fused together by a `SwerveDrivePoseEstimator` (a Kalman filter),
which produces a best-guess `Pose2d` (an x, y, and angle) of where the robot is.
This is updated every loop in `updateOdometry()`. Knowing position is what lets
autonomous drive to exact spots and lets teleop auto-aim work.

### Auto-Aim

When auto-aim is on, the code calculates the angle from the robot to the hub
(`getAngleFromTargetPosition`), compares it to the robot's current heading, and
uses a PID controller to rotate the robot until it's pointed at the hub. The
driver can still drive around; only the rotation is taken over. `isLockedOn()`
reports whether we're aimed within a few degrees.

### Trajectory Following

For smooth curved paths, the drive system can follow a **trajectory** — a
pre-computed path with positions and speeds at every moment in time. A
`HolonomicDriveController` looks at where the robot *should* be at the current
time versus where it actually is, and corrects the difference. Autonomous Lua
scripts can request these paths with `followPath()`.

---

## 8. Subsystem: Intake

**Files:** [src/main/include/IntakeSystem.h](src/main/include/IntakeSystem.h),
[src/main/cpp/IntakeSystem.cpp](src/main/cpp/IntakeSystem.cpp)

The intake grabs game pieces ("fuel") off the floor and feeds them into the
robot. It has two independent parts, each with its own mode/state machine:

### The Feed (rollers)

`FeedMode` can be `STOP`, `INTAKE` (pull pieces in), `OUTTAKE` (spit them out),
or `HALF_SPEED` (gentler intake, used when also feeding the shooter). Two feed
motors run together to grab pieces.

A safety detail: `setFeedVoltage()` refuses to run the rollers if the pivot is
below a "danger height." This prevents the rollers from running while the intake
is folded in, which could jam or damage things.

### The Pivot (arm)

`PivotMode` can be `STOP`, `UP`, `DOWN`, or `WIGGLE`. The pivot is the arm that
raises and lowers the intake. It uses a **PID controller** plus a CANcoder
(angle sensor) to move to a target angle and hold there. The target angles
(`pivot_up_angle`, `pivot_down_angle`) come from the config file.

**Wiggle** is a special mode that gently oscillates the pivot back and forth —
this helps shake stuck game pieces loose so they feed properly.

The pivot power is clamped between a min and max voltage, and is forced to stop
if it tries to go past its physical limits (`pivot_max_angle` /
`pivot_min_angle`).

---

## 9. Subsystem: Shooter

**Files:** [src/main/include/ShooterSystem.h](src/main/include/ShooterSystem.h),
[src/main/cpp/ShooterSystem.cpp](src/main/cpp/ShooterSystem.cpp)

The shooter launches game pieces at the hub. This is the most "physics-heavy"
subsystem. It has two state machines: one for the **shooter wheels** and one for
the **loader** (which feeds pieces up into the wheels).

### Shooter Modes

| Mode | What it does |
|------|--------------|
| `STOP` | Motors off |
| `IDLE` | Spin slowly so the wheels are ready to ramp up quickly |
| `VOLTAGE` | Apply a fixed voltage (used for testing) |
| `SPEED` | Spin to an exact target RPM |
| `AUTO` | **The main mode** — automatically pick speed and angle based on distance to the hub |
| `MANUAL_AUTO` | Like AUTO but with a manually set speed |
| `SNOW_BLOW` | Launch pieces far across the field at low angle |

### AUTO Mode: The Clever Part

When shooting, the right wheel speed and launch angle depend on **how far away
the hub is**. We measured the right values at several distances and stored them
in the config as lookup tables:

```json
"distance_to_shooter_velocity":[
    [2.0, 3500.0],   ← at 2.0 meters, spin 3500 RPM
    [3.0, 4175.0],   ← at 3.0 meters, spin 4175 RPM
    [4.0, 4955.0],
    ...
]
```

For distances *between* the measured points, the code uses **cubic Hermite
interpolation** (a smooth curve through the data points) to estimate the right
value. This happens in `doShooterAutoControl()`, which:
1. Asks the drive system for the current distance to the hub.
2. Looks up the right wheel speed, backspin, and servo angle for that distance.
3. Sets the wheels spinning and aims the servo-controlled hood.

### Backspin and the Two Wheels

The shooter has a left and right wheel that can spin at *slightly different
speeds*. This difference puts **backspin** on the game piece, which makes its
flight more stable and consistent. The amount of backspin also changes with
distance (another lookup table).

### Servos (the hood angle)

Two servos controlled by a `ServoHub` set the angle of the shooter hood. The
angle is sent as a "pulse width" (a servo control signal). Like the speed, the
right angle for each distance comes from a lookup table.

### "At Speed" Check

`isAtSpeed()` checks whether both wheels have actually reached their target RPM
(within a tolerance). Autonomous scripts wait for this to be true before feeding
a piece, so we don't shoot before the wheels are ready. The controller also
rumbles when the shooter is at speed.

### Ramp Limiting

Jumping the shooter from 0 to full speed instantly would draw a huge amount of
current and could **brown out** the robot (drop the battery voltage so low that
things reset). To prevent this, `setShooterVelocityControl()` *ramps* the speed
up gradually using `shooter_ramp_acceleration` from the config.

---

## 10. Subsystem: Climb

**Files:** [src/main/include/ClimbSystem.h](src/main/include/ClimbSystem.h),
[src/main/cpp/ClimbSystem.cpp](src/main/cpp/ClimbSystem.cpp)

The climb subsystem exists as a skeleton (an empty shell ready to be filled in)
but is **not implemented yet** as of this writing. The class and its `Periodic()`
function exist, but they don't do anything. When the team builds a climber
mechanism, the motor control code will go here, following the same subsystem
pattern as the others.

---

## 11. Vision (Cameras and the AprilTags)

**Files:** [src/main/include/Vision.h](src/main/include/Vision.h),
[src/main/cpp/Vision.cpp](src/main/cpp/Vision.cpp)

Our robot uses **Limelight** cameras that detect **AprilTags** — special
black-and-white squares placed around the field at known locations. By seeing a
tag and knowing where that tag is, the camera can calculate where the *robot*
must be. This is a second, independent way of knowing our position that helps
correct drift in the wheel-based odometry.

We have multiple cameras (named `limelight`, `limelight-left`,
`limelight-right`). Each one is processed independently and any good reading is
fed into the same pose estimator.

### Trusting the Cameras (Standard Deviations)

This is the most important idea in the Vision code. The pose estimator is a
**Kalman filter** — it blends wheel odometry and camera readings together. We
tell it *how much to trust* each camera reading using "standard deviations":

- **Lower standard deviation = trust the camera MORE** (pull our position toward
  what the camera says).
- **Higher standard deviation = trust the camera LESS** (rely more on the
  wheels).

The trust level depends on two things:
1. **How many tags are visible** — seeing 2+ tags is much more reliable than 1.
2. **How far away the tags are** — closer tags give a more accurate reading.

The formula is roughly: `std_dev = BASE + (PER_METER × tag_distance)`. All the
`BASE` and `PER_METER` numbers are tuned constants in
[Vision.h](src/main/include/Vision.h).

### Auton vs Teleop Filtering

Vision behaves differently depending on mode:
- **In Autonomous**, it filters strictly. If a camera reading disagrees too much
  with where we think we are, it's *rejected* — a bad reading during auton could
  send the robot to the wrong place. Single-tag readings get especially strict
  checks.
- **In Teleop**, it trusts the cameras much more, because getting bumped by
  other robots can cause big odometry drift that we *want* the camera to correct.

---

## 12. The Lua Scripting System (Autonomous)

This is one of the most unique parts of our codebase, so it gets its own big
section.

### Why Lua?

Autonomous routines (the 15-second self-driving phase) change *constantly*
during build season as the strategy evolves. If every autonomous were written
in C++, every change would mean recompiling and re-deploying the whole robot
program — slow and error-prone.

Instead, autonomous routines are written in **Lua**, a simple scripting
language. The Lua files live in
[src/main/deploy/scripts/autons/](src/main/deploy/scripts/autons/) and are
copied onto the robot like config files. You can edit an autonomous, re-deploy
*just that file*, and try it — no recompile needed.

### How C++ and Lua Talk to Each Other

The bridge between C++ and Lua is in
[src/HighVoltageLuaLib/](src/HighVoltageLuaLib/) and uses a library called
**LuaBridge**. The key class is `Keeko::ThreadedScript`
([ThreadedScript.h](src/HighVoltageLuaLib/include/Keeko/ThreadedScript.h)).

Each subsystem has an `addToLuaState()` function that **exposes** some of its C++
methods to Lua. For example, in the drive system:

```cpp
void DriveSystem::addToLuaState(Keeko::ThreadedScript& script)
{
    luabridge::getGlobalNamespace(script)
        .beginClass<DriveSystem>("DriveSystem")
            .addFunction("driveTo", &DriveSystem::driveToLua)
            .addFunction("rotateTo", &DriveSystem::rotateToLua)
            .addFunction("setOdometry", &DriveSystem::setOdometryLua)
            // ...
        .endClass();
    script.addInstance("drive_system", this);  // name it "drive_system" in Lua
}
```

That last line registers the actual subsystem object under the name
`drive_system`. So in a Lua script you can write `drive_system:driveTo(...)`
and it calls straight into C++.

Notice the functions exposed to Lua are often special `...Lua` versions (like
`driveToLua`). That's because Lua only understands plain numbers, not WPILib's
typed units. The `...Lua` wrappers take plain numbers and convert them into
units before calling the real function.

### What a Lua Autonomous Looks Like

Here's the structure of every auton (simplified from
[QuickLoad.lua](src/main/deploy/scripts/autons/QuickLoad.lua)):

```lua
function autonomous()
    drive_system:setMaxSpeed(3.5)
    drive_system:setOdometry(4.4, 3.40, 180)   -- tell the robot where it starts
    drive_system:driveTo(6.39, 3.36, 180)       -- drive somewhere
    waitUntilDistance(0.0762)                    -- wait until we arrive
    intake_system:pivotDown()
    intake_system:feedIntake()                   -- grab a game piece
    -- ... drive, shoot, repeat ...
    drive_system:stop()
end
```

Three objects are always available inside a script: `drive_system`,
`intake_system`, and `shooter_system`. There's also a `control` object with
helpers like:
- **`control:sleep(seconds)`** — pause the script for some seconds.
- **`control:waitUntilTrue(function)`** — pause until a condition becomes true
  (e.g. wait until the robot is close enough to its target, or until the shooter
  is at speed).

### Threading and Safety

The Lua script runs on its **own separate thread**, not the main robot loop.
This is why it can use `sleep()` and `waitUntilTrue()` without freezing the
robot — the main 50-times-per-second loop keeps running the subsystems while the
script waits.

A few important safety details handled in
[Robot.cpp](src/main/cpp/Robot.cpp):
- The script for the selected auton is loaded ahead of time during **Disabled**
  mode, so there's no lag when auton starts.
- If a script crashes or errors, an `onFailCallback` immediately stops the
  drivetrain so the robot doesn't run away.
- When the script is told to `stop()`, the `sleep`/`waitUntilTrue` helpers throw
  an error to immediately end the script cleanly.

### Picking an Auton

The list of available autons is defined in `Robot.h` as the `AUTONS` vector,
which pairs a display name with a Lua file path (from
[Paths.h](src/main/include/Paths.h)). Drivers pick one from a dropdown on the
dashboard, which is implemented by `StringSelector` (a Network Tables–backed
menu). When the selection changes, `checkIfAutonScriptNeedsLoading()` loads the
new script in the background.

---

## 13. The Config System (Tuning Without Recompiling)

**Files:** [src/main/include/RobotConfig.h](src/main/include/RobotConfig.h),
[src/main/cpp/RobotConfig.cpp](src/main/cpp/RobotConfig.cpp),
[src/main/deploy/config/BaseRobot.json](src/main/deploy/config/BaseRobot.json)

Almost every "magic number" in the robot — motor CAN IDs, PID gains, voltages,
target angles, the shooter distance tables — lives in a **JSON config file**
instead of being hard-coded. This means you can re-tune the robot by editing a
text file and re-deploying, without recompiling the C++.

When the robot boots, `RobotContext::loadConfig()` reads
[BaseRobot.json](src/main/deploy/config/BaseRobot.json) and parses it into C++
structs (`DriveConfig`, `IntakeConfig`, `ShooterConfig`). The parsing is done by
the `from_json()` functions using the **nlohmann/json** library. Each subsystem
then reads its values out of the config in its constructor.

The config is split into three sections matching the subsystems:
- **`drive`** — swerve module IDs, encoder offsets, PID gains, wheel positions.
- **`intake`** — motor IDs, voltages, pivot angles, PID.
- **`shooter`** — motor IDs, the distance lookup tables, gear ratio, ramp rate.

There's also `TestRobot.json` for a separate practice/test robot.

---

## 14. Network Tables (Talking to the Laptop)

**Network Tables** is FRC's system for sending data between the robot and the
laptop dashboard in real time. It's like a giant shared dictionary of
key→value pairs that both the robot and the laptop can read and write.

We wrap it in a friendly helper class called
`HighVoltage::NetworkTableValue<T>`
([NetworkTableValue.h](src/HighVoltageRobotLib/include/NetworkTableValue.h)).
You'll see members like this all over the subsystems:

```cpp
HighVoltage::NetworkTableValue<double> nt_distance_to_hub{"Robot_Orientation/distance_to_hub", 0.0};
```

This does two things:
- **Publishing:** `nt_distance_to_hub.setValue(...)` sends data *out* to the
  dashboard so you can watch it live (great for debugging).
- **Subscribing:** with `setOnUpdate(...)`, you can run a callback whenever
  someone *changes* the value from the dashboard. This is how we **live-tune**
  values like PID gains or voltages without re-deploying — change the number on
  the laptop and the robot reacts instantly.

### Throttling in Competition

Sending thousands of values 50 times a second uses up wireless bandwidth, which
is limited at competitions. So in **competition mode** (set in the config), the
robot only publishes telemetry every Nth loop (the "throttle interval"). You can
see this in each subsystem's `updateNetworkTable()` — it counts loops and skips
most of them when throttled. In testing mode, it publishes every loop for
maximum detail.

### Simulation Data

[NTSimulationData.h](src/main/include/NTSimulationData.h) provides fake data
generators (a blinker, a counter, a sine wave) that publish to Network Tables
when running in **simulation** (no real robot). This is handy for testing the
dashboard without a physical robot.

---

## 15. The HighVoltage Library (Our Reusable Toolbox)

**Folder:** [src/HighVoltageRobotLib/](src/HighVoltageRobotLib/)

This is our team's own library of reusable parts that aren't specific to this
year's robot. Keeping them separate means we can reuse them next year. The two
most important pieces — the swerve module and its coordinator — are explained in
depth below, followed by the templated `NetworkTableValue`, and then a quick tour
of the smaller helpers.

### Swerve, from the bottom up

The drivetrain is built in three layers. From lowest to highest:

```
DriveSystem (subsystem, picks WHAT the robot should do)
    │  "drive northeast at half speed while turning left"
    ▼
SwerveModuleCordinator (the choreographer — splits that into 4 wheel jobs)
    │  "front-left: point to 50°, go 2 m/s"  (× 4 wheels)
    ▼
MK5NSwerveModule  × 4  (one per corner — makes the real motors obey)
    │  voltages
    ▼
TalonFX drive motor + TalonFX rotate motor + CANcoder
```

The key idea: each layer speaks to the layer below it in simpler, more concrete
terms. The DriveSystem thinks about the whole robot. The coordinator thinks about
four wheels. Each module thinks about its own two motors. None of them needs to
know the details of the others.

#### `SwerveModule` — the shared blueprint

**File:** [SwerveModule.h](src/HighVoltageRobotLib/include/SwerveModule.h)

`SwerveModule` is an **abstract base class** — a blueprint that says "every
swerve module must be able to do these things" without saying *how*. It stores
two pieces of "what I want" state, `target_angle` and `target_speed`, and
declares functions every module must provide, like `update()`, `getAngle()`, and
`getModulePosition()`. The `= 0` after those functions makes them **pure
virtual**, meaning a subclass is *required* to fill them in.

Why bother with a blueprint? Because our robots have used different swerve
hardware over the years (you'll also find an `MK4SwerveModule` in the library).
As long as a new module class inherits from `SwerveModule` and implements the
required functions, the `SwerveModuleCordinator` can drive it without changing a
single line — it only ever talks to the blueprint, never the specific hardware.

#### `MK5NSwerveModule` — controlling one corner

**Files:** [MK5NSwerveModule.h](src/HighVoltageRobotLib/include/MK5NSwerveModule.h),
[MK5NSwerveModule.cpp](src/HighVoltageRobotLib/src/MK5NSwerveModule.cpp)

This is the concrete module for our current "MK5N" swerve hardware. Each one owns
three CTRE devices: a **drive** TalonFX (spins the wheel), a **rotate** TalonFX
(points the wheel), and a **CANcoder** (an absolute sensor that reads the wheel's
current angle).

**Reading the angle.** The CANcoder doesn't naturally read zero when the wheel
points "forward" — the magnet inside is mounted at some arbitrary rotation. So
each module has an `encoder_offset` (from the config) that we subtract:

```cpp
units::degree_t getRawAngle() { return get180Range(encoder->GetPosition()... * 360.0_deg); }
units::degree_t getAngle()    { return get180Range(getRawAngle() - offset); }
```

`getAngle()` is the corrected, real-world angle the rest of the code uses.
Calibrating those offsets (one per corner) is a routine part of setting up the
drivetrain — they live in [BaseRobot.json](src/main/deploy/config/BaseRobot.json).

**The "flip" optimization (the clever part).** Say a wheel is pointing at 0° and
you ask it to point at 170° and drive forward. It could rotate 170°… *or* it
could rotate just 10° to face 180° (the opposite direction) and drive
**backward** instead — which gets the same result but with far less turning. The
`update()` function works this out every loop:

```cpp
units::degree_t set_angle         = closestAngle(current_angle, target_angle);
units::degree_t set_angle_flipped = closestAngle(current_angle, target_angle + 180.0_deg);
if(std::abs(set_angle.value()) <= std::abs(set_angle_flipped.value()))
{
    new_target = current_angle + set_angle;  is_forward = true;   // turn normally
}
else
{
    new_target = current_angle + set_angle_flipped;  is_forward = false; // flip + reverse
}
```

It picks whichever requires the smaller turn, then drives forward or backward to
match. This is why the wheels never spin more than 90° to reach any heading.

**Two nested controllers for the rotate motor.** Pointing the wheel accurately
uses `updateTurnMotor()`, which layers two controllers:
- `rotate_angle_pid` turns "how many degrees off am I?" into a desired turning
  *voltage*.
- `rotate_velocity_pid` + a **feed forward** help the motor actually hit the
  turning speed smoothly.

If the wheel is within 1° of its target, the motor is just shut off (no point
fighting tiny errors). The drive motor, meanwhile, is controlled in
`updateDriveMotor()` mostly by a feed forward that converts a desired speed into
a voltage.

**Reporting position for odometry.** `getModulePosition()` returns how far this
wheel has rolled and which way it's pointing. The DriveSystem collects all four
of these to figure out where the robot has moved (see
[the odometry discussion](#odometry-knowing-where-you-are)). Note the distance is
negated here — a hardware convention detail that the DriveSystem's trajectory
code comments on as well.

#### `SwerveModuleCordinator` — the choreographer

**Files:** [SwerveDriveCordinator.h](src/HighVoltageRobotLib/include/SwerveDriveCordinator.h),
[SwerveDriveCordinator.cpp](src/HighVoltageRobotLib/src/SwerveDriveCordinator.cpp)

The coordinator's job is to take **one motion request for the whole robot** and
turn it into **four individual wheel jobs**. The DriveSystem hands it three
numbers — a direction `angle`, a `magnitude` (how fast, 0–1), and a `turn`
amount — and the coordinator decides each wheel's target angle and speed.

(Heads up: "Cordinator" is misspelled in the code. It's spelled that way
everywhere, so you have to match it when you use the class.)

**Field-oriented math.** If field-oriented mode is on, the coordinator first
rotates the requested direction by the robot's current heading
(`field_angle`, fed in from the gyro):

```cpp
if(field_oriented) { angle = get180Range(angle - field_angle); }
```

This is the one line that makes "push the stick away from you = robot goes away
from you" work, no matter which way the robot is turned.

**Mixing translation and turning.** Pure translation would point all four wheels
the same way. Pure turning would point them along a circle (tangent to the robot's
frame — that's what the four pre-computed corner angles, `front_left_angle` etc.,
are for; they're calculated once in the constructor from the robot's dimensions).
To do both at once, the coordinator nudges each wheel's angle off the travel
direction by up to ±45° depending on the `turn` value:

```cpp
units::degree_t turn_angle = units::degree_t{turn * 45.0};
// for each wheel, add or subtract turn_angle depending on which side it's on:
if(std::abs(closestAngle(angle, front_left_angle).value()) >= 90.0)
    front_left->setTargetAngle(angle + turn_angle);
else
    front_left->setTargetAngle(angle - turn_angle);
```

The `>= 90.0` check is what makes wheels on opposite corners cant in opposite
directions, which is exactly what produces a turn while you translate. There's
also a simpler `setRotate()` for spinning in place, which just points every wheel
along its turning circle and spins them together.

**Important:** the coordinator only ever *records* targets by calling
`setTargetAngle()` / `setSpeed()` on each module. Nothing physically moves until
someone calls `coordinator->update()`, which forwards `update()` to all four
modules so they run their control loops. The DriveSystem always calls
`setDrive(...)` (or `setRotate(...)`) followed by `update()`. This is the same
"decide, then act" rhythm you see throughout the codebase.

### `NetworkTableValue<T>` — one template for many types

**File:** [NetworkTableValue.h](src/HighVoltageRobotLib/include/NetworkTableValue.h)

We use `NetworkTableValue` constantly to publish telemetry and to receive
live-tuning changes (see [Section 14](#14-network-tables-talking-to-the-laptop)).
What's interesting is *how one class handles many different data types* — `double`,
`bool`, `std::string`, `std::vector<double>`, and more — without us writing a
separate class for each. The trick is C++ **templates**.

A template is a recipe with a blank in it. When you write
`NetworkTableValue<double>`, the compiler stamps out a real class with the blank
(`T`) filled in as `double`. Write `NetworkTableValue<bool>` and you get a
*different* real class with `T` = `bool`. You write the logic once; the compiler
generates a version for each type you actually use.

**The hard part: each NT type needs different helper classes.** WPILib doesn't
have one generic "topic" — it has `nt::DoubleTopic`, `nt::BooleanTopic`,
`nt::StringTopic`, and so on, each with its own publisher and subscriber. So the
template has to choose the *right* WPILib classes based on `T`. It does this with
`std::conditional_t`, which is a compile-time "if" for picking types:

```cpp
using TopicType =
std::conditional_t<std::is_same_v<T, bool>,        nt::BooleanTopic,
std::conditional_t<std::is_arithmetic_v<T> && !std::is_same_v<T, bool>, nt::DoubleTopic,
std::conditional_t<std::is_same_v<T, std::string>, nt::StringTopic,
std::conditional_t<std::is_same_v<T, std::vector<double>>, nt::DoubleArrayTopic,
... void>>>>;
```

Read it like a chain of "if/else if": *if `T` is `bool`, use `BooleanTopic`; else
if `T` is a number (but not bool), use `DoubleTopic; else if it's a string…* The
matching `PublisherType` and `SubscriberType` are chosen the same way. Notice
`bool` has to be checked *first* and explicitly excluded from the "arithmetic"
branch, because in C++ a `bool` technically counts as an arithmetic type — without
that guard, every bool would wrongly become a `DoubleTopic`.

**Choosing behavior, not just types: `if constexpr`.** Picking the type is only
half the battle — the *code* that reads a value out of an NT event also differs
per type (a double calls `.GetDouble()`, a string calls `.GetString()`, an array
loops over elements). The listener uses `if constexpr`, which is an `if` the
compiler evaluates *while compiling*. Only the matching branch gets compiled into
each version of the class; the others are thrown away:

```cpp
if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) {
    if(event...value.IsDouble()) { onUpdate(static_cast<T>(...GetDouble())); }
}
else if constexpr (std::is_same_v<T, std::string>) {
    if(event...value.IsString()) { onUpdate(std::string(...GetString())); }
}
// ... a branch for bool, vector<double>, vector<string> ...
```

This matters because the branches aren't interchangeable — calling `.GetDouble()`
on the string version wouldn't even compile. A normal runtime `if` would try to
compile *all* branches for *every* type and fail. `if constexpr` compiles only the
one that applies.

**Two ways to use it:**
- **Publish (robot → dashboard):** construct with a key and initial value, then
  call `setValue(...)` whenever you want to send data out.
- **Subscribe (dashboard → robot):** call `setOnUpdate(callback)`. Internally an
  NT *listener* fires your callback whenever someone changes the value from the
  laptop. This is exactly how PID gains and voltages get re-tuned live without a
  re-deploy — for example, the shooter re-applies its gains to both motors the
  instant `shooter_kP` changes on the dashboard.

### The smaller helpers

- **`SmartPID`** — a PID controller wrapper that can be tuned live over Network
  Tables (built on `NetworkTableValue`).
- **`StringSelector`** — a dropdown menu backed by Network Tables. It publishes
  the list of options and the current selection, and fires a callback when the
  driver picks a new one. Used for the autonomous chooser.
- **`Utility.h`** — small math helpers used everywhere:
  - `get180Range` / `get360Range` — wrap an angle into a standard range so, e.g.,
    190° becomes -170°. (The swerve code leans on these constantly.)
  - `closestAngle` — the shortest signed turn from one angle to another; it's the
    heart of the module's flip optimization.
  - `clamp` — keep a value between a min and max.
  - `deadZone` — ignore tiny joystick inputs near zero.
  - `linearInterpolation` / `cubicHermiteInterpolation` — look up a value from a
    table of data points (used by the shooter for distance→speed). These are
    *also* templated on input and output unit types.
  - `toDegrees` / `toRadians` — angle conversions.
- **`CommonConfig.h`** — shared config structs (`PIDConfig`,
  `FeedForwardConfig`, etc.) and helpers for applying gains to motors, including
  the RPM→rotations-per-second conversion the shooter needs.
- **`AudioPlayer`**, **`Debounce`**, **`CSVLogger`** — extra utilities for sound,
  cleaning up noisy button presses, and logging data to files.

---

## 16. Glossary

- **AprilTag** — A printed marker on the field that cameras recognize to figure
  out the robot's position.
- **Brownout** — When the battery voltage drops too low (usually from drawing
  too much current at once) and the robot's electronics reset or shut off.
- **CAN ID** — A unique number identifying each motor/sensor on the robot's CAN
  bus (the wiring network that connects them).
- **Feed Forward** — A physics-based "best guess" of the power needed, applied
  before PID correction.
- **Field-Oriented** — Driving where "forward" means away from the driver,
  regardless of which way the robot faces.
- **Kalman Filter** — Math that blends multiple noisy measurements (wheels +
  cameras) into one best estimate.
- **Limelight** — The brand of smart camera we use for vision.
- **Lua** — A lightweight scripting language we use to write autonomous routines.
- **Network Tables** — FRC's real-time data-sharing system between robot and
  laptop.
- **Odometry** — Tracking the robot's position by measuring wheel movement and
  heading.
- **PID Controller** — Math that smoothly drives an error to zero; tuned with
  `kP`, `kI`, `kD`.
- **Periodic** — A function that runs automatically 50 times per second.
- **Pigeon** — Our gyroscope sensor; tells us which way the robot is facing.
- **Pose2d** — A position on the field: x, y, and an angle.
- **RPM** — Revolutions per minute; how we measure shooter wheel speed.
- **State Machine** — A design where code records a "mode" and acts on it every
  loop, instead of acting immediately.
- **Subsystem** — A self-contained class controlling one mechanism of the robot.
- **Swerve Drive** — A drivetrain where each wheel can point and spin
  independently, allowing motion in any direction.
- **TalonFX** — The brand of motor controller used for our motors.
- **Trajectory** — A pre-planned smooth path with positions and speeds over
  time.
- **Units (typed)** — WPILib values that carry their unit (meters, degrees,
  volts) to prevent mix-ups.

---

*This guide describes the code as it exists in the 2026 project. As the robot
evolves, keep this document up to date so the next group of students can learn
from it too.*
