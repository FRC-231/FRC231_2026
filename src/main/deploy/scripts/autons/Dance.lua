local math = require("math")

print("Dance autonomous script loading")

function waitUntilRotationError(angle)
    control:waitUntilTrue( function ()
            return math.abs(drive_system:getRotationError()) < angle
        end)
end

function autonomous()
    print("Dance autonomous script running")
    drive_system:setMaxSpeed(1.5)
    drive_system:setMinPercentage(0.0)
    drive_system:rotateTo(90)
    waitUntilRotationError(1)
    control:sleep(0.5)
    drive_system:rotateTo(-90)
    waitUntilRotationError(1)
    control:sleep(0.5)
    drive_system:rotateTo(0)
    waitUntilRotationError(1)
    control:sleep(0.5)
    drive_system:stop()
end


print("Dance autonomous script loaded")