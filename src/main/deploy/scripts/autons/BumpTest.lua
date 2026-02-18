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

function autonomous()
    print("BumpTest autonomous script running")
    drive_system:setMaxSpeed(2.0)
    drive_system:setMinPercentage(0.0)
    drive_system:setOdometry(6.10, 1.57, 0.0)
    drive_system:rotateTo(35)
    waitUntilRotationError(2)
    drive_system:driveTo(2.59, 1.57, 35)--Over 1st bump
    waitUntilDistance(0.0127)
    drive_system:driveTo(2.59, -1.57, -35)--Drive to side
    waitUntilDistance(0.0127)
    drive_system:driveTo(6.10, -1.57, -35)--Over 2nd bump
    waitUntilDistance(0.0127)
    drive_system:driveTo(6.10, 1.57, 0.0)--Back to OG position
    waitUntilDistance(0.0127)
    drive_system:setMaxSpeed(3.0)
    drive_system:setMinPercentage(0.25)
    drive_system:driveTo(6.10, 3.25, 0.0)--Drive to trench side
    waitUntilDistance(0.05)
    drive_system:driveTo(2.84, 3.25, 0.0)--Drive thru trench
    waitUntilDistance(0.05)
    drive_system:driveTo(2.84, -3.34, 0.0)--Drive to opp. side
    waitUntilDistance(0.05)
    drive_system:driveTo(6.10, -3.34, 0.0)--Drive thru 2nd trench
    waitUntilDistance(0.05)
    drive_system:setMinPercentage(0.0)
    drive_system:driveTo(6.10, 1.57, 0.0)--Back to OG
    waitUntilDistance(0.05)
    drive_system:stop()
end


print("BumpTest autonomous script loaded")