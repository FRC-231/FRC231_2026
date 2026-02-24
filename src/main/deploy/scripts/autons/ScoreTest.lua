local math = require("math")
print("BumpTest autonomous script loading")

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

bump_speed = 2.0
rotate_speed = 4.5
travel_speed = 3.0
intake_speed = 2.0
trench_y = 2.9
function autonomous()
    drive_system:setMaxSpeed(bump_speed)
    drive_system:setMinPercentage(0.0)
    drive_system:setOdometry(3.53, 1.48, 45.0)--Starting position at bump
    drive_system:driveTo(6.32, 1.48, 45.0)--Drive over bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)--Increase speed for rotate
    drive_system:rotateTo(0)--Face the balls
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(7.66, 1.48, 0.0)--Drive to balls
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(6.32, 1.48, -45.0)--Back to position after bump
    waitUntilDistance(0.0762)
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(-45.0)--Rotate to go over bump
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(2.51, 1.48, -45.0)--Go to shooting position
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(0)--Rotate to face tags
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(2.51, trench_y, 0)--Drive to trench side
    waitUntilDistance(0.1)
    drive_system:driveTo(7.0, trench_y, 0)--Go under trench
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(-45.0)--Ensure angle
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(7.60, 0.79, -45.0)--Drive to balls
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(6.48, trench_y, 0.0)--Go back to trench side
    waitUntilDistance(0.0762)
    drive_system:driveTo(2.51, trench_y, 0.0)--Back to position on alliance trench side
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(-45.0)--Ensure angle to face tags
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(2.51, 1.48, -45.0)--Drive to shooting position
    waitUntilDistance(0.0762)

    drive_system:stop()
end