print("CurveTest autonomous script loading")

function waitUntilPathFinished()
    control:waitUntilTrue(function()
        return drive_system:isTrajectoryDone()
    end)
end

function autonomous()
    print("CurveTest autonomous script running")

    -- Snake path from (2.25, 2.0) to (2.25, -2.0)
    -- Snakes left and right with 1m amplitude as it travels in -Y
    --
    --    (2.25, 2.0)   <- start
    --   /
    -- (1.25, 1.0)      <- curve left
    --   \
    --    (3.25, 0.0)   <- curve right
    --   /
    -- (1.25, -1.0)     <- curve left
    --   \
    --    (2.25, -2.0)  <- end

    drive_system:setOdometry(2.25, 2.0, 0.0)
    drive_system:setMaxSpeed(3.0)

    -- Snake path: current position -> left -> right -> left -> end
    -- End angle 0° (robot faces -Y direction)
    drive_system:followPath(4.0,
        {{3.25, 1.0}, 
        {1.25, 0.5}, 
        {2.25, -1.5}, 
        {2.25, -2.0}},  -- waypoints + end point
        0.0             -- end angle
    )
    waitUntilPathFinished()
    drive_system:setMaxSpeed(1.5)
    drive_system:followPath(2.0,
        {{3.36, 1.23},
        {2.27, 3.23},
        {2.42, 1.23},
        {2.25, -2.0}},
        0.0
    )
    waitUntilPathFinished()
    drive_system:stop()
    print("CurveTest autonomous script finished")
end

print("CurveTest autonomous script loaded")
