print("CurveTest autonomous script loading")

function autonomous()
    print("CurveTest autonomous script running")

    -- Orbit center: (2.25, 2.0), radius: 1.0m
    -- Start at the bottom of the circle: (2.25, 1.0)
    -- Travel counter-clockwise: bottom -> right -> top -> left -> bottom
    --
    --         (2.25, 3.0)  <- top
    --        /            \
    -- (1.25, 2.0)      (3.25, 2.0)
    --        \            /
    --         (2.25, 1.0)  <- start/end

    drive_system:setOdometry(2.25, 1.0, 0.0)
    drive_system:setMaxSpeed(2.0)

    -- Single trajectory: start at bottom, pass through right/top/left, end at bottom
    -- Start heading is 0° (facing +X, tangent to circle going right)
    -- End heading is 0° to close the loop smoothly
    drive_system:followTrajectory(2.5,
        2.25, 1.0, 0.0,                          -- start: x, y, heading (tangent direction)
        {{3.25, 2.0}, {2.25, 3.0}, {1.25, 2.0}}, -- interior waypoints: right, top, left
        2.25, 1.0, 0.0                            -- end: back to start
    )
    control:waitUntilTrue(function()
        return drive_system:isTrajectoryDone()
    end)

    drive_system:stop()
    print("CurveTest autonomous script finished")
end

print("CurveTest autonomous script loaded")
