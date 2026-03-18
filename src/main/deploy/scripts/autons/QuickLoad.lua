local math = require("math")
print("QuickLoad autonomous script loading")

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
rotate_speed = 3.5
travel_speed = 2.0
intake_speed = 2.0

function autonomous()
    drive_system:setMaxSpeed(bump_speed)
    drive_system:setMinPercentage(0.0)
    drive_system:setOdometry(3.53, 1.48, 135)--Starting position at bump
    drive_system:driveTo(6.32, 1.48, 135)--Drive over bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(7.76, 1.48, 135)--Drive to fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(90)
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(7.76, -1.65, 90)--Face the balls as we drive across
    waitUntilDistance(0.0762)
    waitUntilRotationError(2)
    drive_system:driveTo(6.27, -1.65, 225)--Drive in front of bump

end
