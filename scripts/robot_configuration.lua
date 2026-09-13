-- [ Robot Configuration Script ]
-- Title:     Robot Mode Configuration
-- CoppeliaSim: Educational 4.10
-- Author:    io4nn1s
-- Year:      2025-2026
--
-- NOTE:
-- This script is used once to configure the robot and its
-- required components for kinematic operation. It disables
-- the relevant dynamic properties to avoid performing this
-- configuration manually.
--
-- This script is normally not executed during simulation.

sim = require('sim')

-- ================= CONFIGURATION =================
-- Change to "Kinematic" or "Dynamic"
local defaultMode = "Kinematic"

-- Name of the robot base
local robotBaseName = "Robot_Sawyer"

-- ================= MAIN FUNCTION =================
-- To use it, choose the Customization script next to
-- the console and enter the function with the right
-- parameters, e.g. setRobotMode("Kinematic", "UR10")
--
function setRobotMode(mode, robotName)
    if mode ~= "Kinematic" and mode ~= "Dynamic" then
        print("Invalid mode! Use 'Kinematic' or 'Dynamic'")
        return
    end

    local modeConst = (mode == "Dynamic") and 1 or 0

    -- get root object
    local root = sim.getObject('/'..robotName)
    if root == -1 then
        print("Robot base not found:", robotName)
        return
    end

    -- get all objects under root
    local allObjs = sim.getObjectsInTree(root)

    for i=1,#allObjs do
        local h = allObjs[i]
        local name = sim.getObjectAlias(h,0)

        if h ~= root then
            local t = sim.getObjectType(h)

            -- Set joints
            if t == sim.object_joint_type then
                if mode == "Dynamic" then
                    sim.setJointMode(h, sim.jointmode_force)  -- Dynamic
                else
                    sim.setJointMode(h, sim.jointmode_passive)  -- Kinematic
                end
                print(string.format("Joint '%s' set to %s", name, mode))
            end

            -- Set shapes
            if t == sim.object_shape_type then
                local ok, err = pcall(function()
                    sim.setObjectInt32Parameter(h, sim.shapeintparam_static, (modeConst==1) and 0 or 1)
                end)
                if ok then
                    print(string.format("Shape '%s' set to %s", name, mode))
                end
            end
        end
    end

    print("All eligible objects updated to " .. mode)
end

-- Make global so it can be called from console
_G["setRobotMode"] = setRobotMode

-- ================= OPTIONAL =================
-- Uncomment the next line if you want it to run automatically during start of simulation
 --setRobotMode(defaultMode)
