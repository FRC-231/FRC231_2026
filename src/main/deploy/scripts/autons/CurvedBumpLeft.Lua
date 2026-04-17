local math = require("math")
print("CurvedDoubleTakeLeft autonomous script loading")

function waitUntilPathFinished()
    control:waitUntilTrue(function()
        return drive_system:isTrajectoryDone()
    end)
end

function waitUntilRotationError(angle)
    control:waitUntilTrue( function ()
            return math.abs(drive_system:getRotationError()) < angle
        end)
end

function waitUntilDistance(distance)
    control:waitUntilTrue( function ()
            return drive_system:getDistanceFromTarget() < distance
        end)
end

--Auton speeds
bump_speed = 2.5
rotate_speed = 3.5
travel_speed = 3.5
intake_speed = 2.5

--Coordinates
trench_y = 3.30
bump_y = 1.47

function autonomous()
    print("CurvedDoubleTakeLeft autonomous script running")
    drive_system:setOdometry(3.36, bump_y, -45.0) --Start at the bump, angled -45.0
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(6.20, bump_y, -45.0) --Drive over bump
    waitUntilDistance(0.0762)
    drive_system:rotateTo(180.0) -- Rotate so shooter faces driverstation
    waitUntilRotationError(2)
    intake_system:pivotDown()
    control:sleep(0.5)
    intake_system:feedIntake()
    drive_system:setMaxSpeed(intake_speed)
    drive_system:followPath(4.0,
    {{7.15, 1.20},
    {7.65, 0.60},
    {7.80, -0.20}},
    90.0)
    waitUntilPathFinished() --Curve towards the fuel in neutral zone
    intake_system:feedStop()
    intake_system:pivotUp()
    drive_system:driveTo(7.75, bump_y, 90.0) --Align with bump
    waitUntilDistance(0.0762)
    drive_system:rotateTo(-45)
    waitUntilRotationError(2)
    drive_system:driveTo(5.9, bump_y, -45.0) --Drive in front of bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(bump_speed)
    shooter_system:autoShoot()--Start shooter wheels before going over bump
    drive_system:driveTo(3.2, bump_y, -45.0) --Drive across bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(2.5, 1.6, -45.0)-- Drive to shooting position
    waitUntilDistance(0.0762)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.75)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()--Activate hopper
    intake_system:feedIntake()
    control:sleep(3.0) --This is how long we're shooting
    shooter_system:loaderStop()
    intake_system:feedStop()
    shooter_system:stopShooter()
    drive_system:driveTo(3.2, 1.52, -45.0) --Drive in front bump
    waitUntilDistance(0.0762)
    drive_system:rotateTo(150)
    waitUntilRotationError(2)
    control:sleep(0.5)
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(6.5, 1.52, 150.0) --Drive across of bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(6.9, bump_y, 150.0) --Drive in front of bump
    waitUntilDistance(0.0762)
    intake_system:pivotDown()
    control:sleep(0.5)
    intake_system:feedIntake()
    drive_system:setMaxSpeed(intake_speed)
    drive_system:followPath(4.0,
    {{6.9, 0.4},
    {7.0, -0.45},
    {7.2, -0.8},
    {7.7, -0.6}},
    -175.0)
    waitUntilPathFinished()
    drive_system:followPath(4.0,
    {{8.25, -0.85},
    {8.95, 0.35},
    {8.9, 1.4},
    {8.6, 1.75}},
    -70.0)
    waitUntilPathFinished()

    drive_system:stop()
end