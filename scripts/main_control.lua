-- [ Main Control Script ]
-- Title:     Motion Planning and Collision Avoidance in CoppeliaSim Using OMPL
-- Subtitle:  An Industrial Manipulation Case Study
-- Robot:     Sawyer Manipulator
-- Author:    io4nn1s
-- Year:      2025-2026

sim     = require 'sim'     -- Main Simulation Library
simIK   = require 'simIK'   -- IK Library
simOMPL = require 'simOMPL' -- OMPL Library
simUI   = require 'simUI'   -- UI Library (optional pop-up window)

------------------------------------------------------------------
-- BASIC CONFIGURATION FUNCTIONS
------------------------------------------------------------------

-- Get current robot joint configuration
function getConfig()
    local config = {}
    for i = 1, #simJointHandles do
        config[i] = sim.getJointPosition(simJointHandles[i])
    end
    return config
end

-- Set robot joint configuration
function setConfig(config)
    for i = 1, #simJointHandles do
        sim.setJointPosition(simJointHandles[i], config[i])
    end
end

-- Collision-checking callback for IK / OMPL
function configurationValidationCallback(config)
    local tmp = getConfig()
    setConfig(config)
    local collisionFree = sim.checkCollision(robotCollection, sim.handle_all) == 0
    setConfig(tmp)
    return collisionFree
end

------------------------------------------------------------------
-- PATH PLANNING & EXECUTION
------------------------------------------------------------------

--- Executes an OMPL-generated joint path
-- @param task OMPL task handle
-- @param path OMPL path handle
-- @param drawPath boolean, enables end-effector path visualization
function executePath(task, path, drawPath)
    local sw = sim.setStepping(true)
    local lastPos = nil

    for i = 1, simOMPL.getPathStateCount(task, path) do
        local conf = simOMPL.getPathState(task, path, i)
        setConfig(conf)

        -- Optional path visualisation
        if drawPath and (i % 3 == 0) then
            local newPos = sim.getObjectPosition(simTip, -1)
            if lastPos then
                sim.addDrawingObjectItem(drawingHandle, {
                    lastPos[1], lastPos[2], lastPos[3],
                    newPos[1], newPos[2], newPos[3]
                })
            end
            lastPos = newPos
        end

        sim.step()
    end

    sim.setStepping(sw)
end

--- Plans and executes a collision-free motion to a target pose
-- @param targetHandle handle of the target dummy
-- @param algorithm OMPL planning algorithm
-- @param drawPath boolean, enables path visualization
-- @return boolean success status
function moveToTarget(targetHandle, algorithm, drawPath)

    -- Set IK target pose
    local pose = sim.getObjectPose(targetHandle, simBase)
    simIK.setObjectPose(ikEnv, ikTarget, pose, ikBase)

    -- Find collision-free IK solutions
    local configs = simIK.findConfigs(
        ikEnv, ikGroup, ikJointHandles,
        {maxDist = 0.85, cb = configurationValidationCallback}
    )

    if #configs == 0 then
        print("No IK configurations found!") -- debugging message (so why know if IK failed)
        return false
    end
    print("IK configs found:",#configs)
    
    -- Choose IK solution closest to current robot configuration
    local goalConfig = selectClosestConfig(configs)
    -- Extra safety net if selectClosestConfig returns nil
    if not goalConfig then
        simOMPL.destroyTask(task)
        return false
    end

    -- Plan OMPL path
    local task = simOMPL.createTask('motionTask')
    simOMPL.setAlgorithm(task, algorithm)
    simOMPL.setStateSpaceForJoints(task, simJointHandles, useForProjection)
    simOMPL.setCollisionPairs(task, {robotCollection, sim.handle_all})
    simOMPL.setStartState(task, getConfig())
    simOMPL.setGoalState(task, goalConfig)   -- configs[1])
    simOMPL.setup(task)

    local maxTime = 4            -- maximum planning time (seconds)
    local simplifyTime = -1      -- no path simplification time limit
    local maxIterations = 300    -- maximum planner iterations
    
    -- Use OMPL to find a solution for the given motion planning task
    local res, path = simOMPL.compute(task, maxTime, simplifyTime, maxIterations)

    if res and path then
        executePath(task, path, drawPath)
    end

    simOMPL.destroyTask(task)  -- we do this to compute a new task next time
    return res and path
end

--- Move robot back to its initial (home) configuration
-- @param drawPath Enable/Disable path visualization
function moveToHome(drawPath)
    local task = simOMPL.createTask('homeTask')
    simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRTConnect)  -- Using just RRTConnect
    simOMPL.setStateSpaceForJoints(task, simJointHandles, useForProjection)
    simOMPL.setCollisionPairs(task, {robotCollection, sim.handle_all})
    simOMPL.setStartState(task, getConfig())
    simOMPL.setGoalState(task, homeConfig)
    simOMPL.setup(task)

    local maxTime = 2            -- maximum planning time (seconds)
    local simplifyTime = -1      -- no path simplification time limit
    local maxIterations = 300    -- maximum planner iterations
    
    -- Use OMPL to find a solution for the given motion planning task
    local res, path = simOMPL.compute(task, maxTime, simplifyTime, maxIterations)

    if res and path then
        executePath(task, path, drawPath)
    end

    simOMPL.destroyTask(task)
    return res and path
end

------------------------------------------------------------------
-- PICK & PLACE FUNCTIONS
------------------------------------------------------------------

--- Moves robot to pick target and attaches the object to the end-effector
-- @param pickTarget handle of the pick target dummy
-- @param objectHandle handle of the object to pick
-- @param drawPath boolean, enables path visualization
-- @return boolean success status
function pickObject(pickTarget, objectHandle, drawPath)
    -- First check if the object handle is valid
    if not objectHandle or not sim.isHandle(objectHandle) then
        print('[pickObject] Invalid objectHandle!')
        return false
    end
    
    -- Move using an OMPL planner algorithm
    local success = moveToTarget(pickTarget,
                                        -- DESCRIPTION:
        --simOMPL.Algorithm.SBL,        -- Bidirectional lazy planner; works well in moderately cluttered spaces
        simOMPL.Algorithm.RRTConnect,   -- Fast bidirectional planner; good for open spaces, weaker in narrow passages
        --simOMPL.Algorithm.KPIECE1,    -- Cell-based planner; better for tight, constrained workspaces
        --simOMPL.Algorithm.BKPIECE1,   -- Biased KPIECE variant; explores narrow regions more aggressively
        --simOMPL.Algorithm.RRTstar,    -- Optimal RRT planner; improves path quality over time but slower   
        drawPath
    )
    if not success then   -- exit if robot fails to reach object
        print('[pickObject] Failed to plan motion!')
        return false
    end
    
    -- Attach object to robot tip
    sim.setObjectParent(objectHandle, simTip, true)
    sim.setObjectPosition(objectHandle, {0, 0, 0}, simTip)   -- set obj. position to match robot tip (simTip)
    sim.setObjectOrientation(objectHandle,                   -- set obj. orientation relevant to robot tip
       {math.rad(0), math.rad(0), math.rad(0)},              -- or set custom orientation in degrees (converted to rad, optional)
       simTip)                                               -- NOTE: tip should match target orientation if IK correct

    return true
end

--- Moves robot to place target and detaches the object
-- @param placeTarget handle of the place target dummy
-- @param objectHandle handle of the object to place
-- @param drawPath boolean, enables path visualization
-- @return boolean success status
function placeObject(placeTarget, objectHandle, drawPath)
    -- First check if the object handle is valid
    if not objectHandle or not sim.isHandle(objectHandle) then
        print('[placeObject] Invalid objectHandle!')
        return false
    end
    
    -- Move using an OMPL planner algorithm
    local success = moveToTarget(
        placeTarget,                    -- FEEDBACK (after testing):
        simOMPL.Algorithm.SBL,          -- better for this cluttered scenario (BEST)
        --simOMPL.Algorithm.RRTConnect, -- good, but sometimes makes unnecessarily complex moves
        --simOMPL.Algorithm.KPIECE1,    -- alright, but sometimes fails to reach target or correct pose
        --simOMPL.Algorithm.BKPIECE1,   -- good, similar responce to RRTConnect, but much slower
        --simOMPL.Algorithm.RRTstar,    -- constantly fails to reach targets (WORSE)
        
        drawPath
    )
    if not success then
        print('[placeObject] Failed to plan motion!')
        return false
    end

    -- Detach object at place location
    sim.setObjectParent(objectHandle, -1, true)
    -- Set object to match target Position
    objPos = sim.getObjectPosition(placeTarget, sim.handle_world)
    sim.setObjectPosition(objectHandle, objPos, sim.handle_world)
    
    return true
end

------------------------------------------------------------------
-- OPTIMIZATION FUNCTIONS
------------------------------------------------------------------

--- Selects the IK configuration closest to the current joint state
-- Minimizes joint-space distance to ensure smooth, consistent motion
-- @param configs table of IK joint configurations
-- @return table selected joint configuration
function selectClosestConfig(configs)

    -- Safety check: no IK solutions available
    if not configs or #configs == 0 then
        print('[selectClosestConfig] No configurations provided')
        return nil
    end
    
    local current = getConfig()    -- get current configuration
    local bestConfig = configs[1]  -- choose 1st config by default
    local bestIndex = 1            -- store index for reference
    local bestDist = math.huge     -- Initialise the best (minimum) distance with a very large number

    -- Assuming 1 or more configs found
    for i = 1, #configs do
        local d = 0
        for j = 1, #current do                          -- Loop over each joint in the robot configuration
            local diff = configs[i][j] - current[j]     -- Difference between joint j of IK solution and current robot joint position
            d = d + diff * diff                         -- Accumulate squared joint-space distance (Euclidean metric, without sqrt)
        end
        if d < bestDist then                            -- Store configuration with the shortest distance
            bestDist = d
            bestConfig = configs[i]
            bestIndex = i
        end
    end

    -- Debugging / inspection (can be removed later)
    print('[selectClosestConfig] Selected config:', bestIndex)
    return bestConfig
end

------------------------------------------------------------------
-- SCENE CONFIGURATION
------------------------------------------------------------------
--- Clones an object and places it at a given spawn location,
--- preserving the template's original (default) orientation.
-- @param templateObject Handle of the template object to clone
-- @param spawnTarget Handle of the spawn reference target
-- @return newObj Handle of the newly spawned object
function spawnObjectDefault(templateObject, spawnTarget)

    -- Clone object from template (deep copy, no parent)
    local newObj = sim.copyPasteObjects({templateObject}, 0)[1]

    -- Read position from spawn target (world frame)
    local pos = sim.getObjectPosition(spawnTarget, sim.handle_world)

    -- Place cloned obeject to spawn target location (position only)
    sim.setObjectPosition(newObj, pos, sim.handle_world)
    
    -- Enable dynamics on the clone object
    setObjectDynamic(newObj, true)

    return newObj
end

--- Enables or disables object dynamics.
-- Used by the state machine to switch between physics-based transport
-- and kinematic manipulation.
-- @param objectHandle Handle of the object
-- @param enabled true to enable dynamics, false to freeze the object
function setObjectDynamic(objectHandle, enabled)

    if not objectHandle or objectHandle == -1 then
        return
    end

    if enabled then
        -- Enable dynamics
        sim.resetDynamicObject(objectHandle)   -- reset to ensure proper responce
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_static, 0)
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_respondable, 1)
    else
        -- Disable dynamics (freeze object)
        sim.resetDynamicObject(objectHandle)   -- reset to ensure proper responce
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_respondable, 0)
    end
end

------------------------------------------------------------------
-- SIMULATION UI (START-UP WINDOW)
------------------------------------------------------------------
-- Enables pop-up window at the start of simulation to allow users
-- to choose which objects they want to spawn. This makes it easier
-- to test specific placements of objects instead of waiting for
-- every other object to be placed.
function createSpawnUI()
    local xml = [[
    <ui title="Spawn Configuration" 
        closeable="true" 
        resizable="false" 
        modal="true"
        on-close="onUIClose">
        
        <label text="Choose which objects you want to spawn
and click 'Run Simulation' to continue."/>
        
        <group layout="vbox">
            <label text="Select objects to spawn:"/>

            <checkbox id="1" text="Cuboid (placed on shelves)" checked="true"/>
            <checkbox id="2" text="Cylinder (placed on exit conveyor)" checked="true"/>
            <checkbox id="3" text="Sphere (placed in bin)" checked="true"/>

            <button text="Run Simulation" on-click="onRunClicked"/>
        </group>
        <label text="Note: If no objects selected, only cuboids spawn"/>
    </ui>
    ]]
    uiHandle = simUI.create(xml)
end

-- Runs when the user clicks the "Run Simulation" button from the UI above
function onRunClicked(ui)
    -- Create item list (global)
    item = {}

    -- Insert selected objects to the item list
    if simUI.getCheckboxValue(ui, 1) ~= 0 then
        table.insert(item, cuboid)
    end
    if simUI.getCheckboxValue(ui, 2) ~= 0 then
        table.insert(item, cylinder)
    end
    if simUI.getCheckboxValue(ui, 3) ~= 0 then
        table.insert(item, sphere)
    end

    -- If no object selected, spawn cuboids only
    if #item == 0 then
        item = {cuboid}  -- default object
    end

    -- Close window and resume simulation
    simUI.destroy(ui)
    uiHandle = nil
    startSimulation = true
end

-- Runs if user clicks the 'X' button on the UI window
function onUIClose(ui)
    -- Destroy UI safely
    if ui then
        simUI.destroy(ui)
    end
    -- Clear flags and stop simulation entirely
    uiHandle = nil
    startSimulation = false
    sim.stopSimulation()
end

--==================================================================--
--         MAIN THREAD
--==================================================================--

function sysCall_thread()

    ------------------------------------------------------------------
    -- GET ALL SCENE HANDLES
    ------------------------------------------------------------------
    
    -- Robot base and joints
    simBase = sim.getObject('..')
    simJointHandles = {}
    useForProjection = {}
    for i = 1, 7 do
        simJointHandles[i] = sim.getObject('../joint', {index = i - 1})
        useForProjection[i] = (i <= 3) and 1 or 0
    end

    -- Disable on-board cameras
    sim.setExplicitHandling(sim.getObject('../head_camera'), 1)
    sim.setExplicitHandling(sim.getObject('../wristCamera'), 1)

    -- Tip and IK target
    simTip    = sim.getObject('../tip')
    simTarget = sim.getObject('../targetIK')

    -- Pick and place targets
    pickTarget = sim.getObject('/pickTarget')
    
    targetShelves = {
        sim.getObject('/targetShelf1'),
        sim.getObject('/targetShelf2'),
        sim.getObject('/targetShelf3'),
        sim.getObject('/targetShelf4')
    }
    shelfIndex = 1        -- current shelf slot
    shelfObjects = {}     -- handles of objects placed on the shelf
    
    targetBin = sim.getObject('/targetBin')
    targetExit = sim.getObject('/targetExit')
        
    -- Object spawning target
    local spawnTarget = sim.getObject('/spawnPoint')

    -- Objects to manipulate 
    sphere = sim.getObject('/Sphere')
    cuboid = sim.getObject('/Cuboid')
    cylinder = sim.getObject('/Cylinder')

    -- Conveyor belt handle
    conveyorBelt = sim.getObject('/conveyorSystem')
    
    -- Proximity sensor for detecting conveyor items
    pickSensor = sim.getObject('/pickSensor')
    
    ------------------------------------------------------------------
    -- IK SETUP
    ------------------------------------------------------------------
    
    -- Robot collection for collision checking
    robotCollection = sim.createCollection()
    sim.addItemToCollection(robotCollection, sim.handle_tree, simBase, 0)

    -- IK environment setup
    ikEnv   = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)

    local _, map = simIK.addElementFromScene(
        ikEnv, ikGroup, simBase, simTip, simTarget, simIK.constraint_pose
    )

    ikJointHandles = {}
    for i = 1, #simJointHandles do
        ikJointHandles[i] = map[simJointHandles[i]]
    end

    ikTarget = map[simTarget]
    ikBase   = map[simBase]

    -- Drawing object for end-effector path (if drawing enabled - optional)
    drawingHandle = sim.addDrawingObject(
        sim.drawing_lines, 2, 0, -1, 2000, {1, 0, 0}
    )
    
    -- Get initial robot joint configuration
    homeConfig = getConfig()
    
    if not simUI then
        error('simUI module not available')
    end

    -- UI Pop-up
    startSimulation = false
    createSpawnUI()
    
    while not startSimulation do
        sim.wait(0.1)
    end

    
    ------------------------------------------------------------------
    -- TASK EXECUTION (State Machine)
    ------------------------------------------------------------------  
    -- States
    local STATE_WAITING  = 0   -- waiting for object to arrive
    local STATE_PICKING  = 1   -- attempting pick
    local STATE_PLACING  = 2   -- placing object
    local STATE_SPAWNING = 3   -- spawn next object
    ------------------------------------------------------------------
    
    -- Initialize scene and spawn first object
    local state = STATE_WAITING
    print('-> STATE_WAITING')   -- status message
    local objIndex= 1           -- set index
    activeObject = spawnObjectDefault(item[objIndex], spawnTarget)
    if activeObject then
        print('Spawned object')
    end

    -- Start conveyor
    sim.writeCustomTableData(conveyorBelt, '__ctrl__', {vel = 0.6})
    
    -- Note to self: avoid using sim.setBufferProperty in threaded scripts
    
    while true do
        ------------------------------------------------------------------
        -- WAIT FOR OBJECT ON CONVEYOR
        ------------------------------------------------------------------
        if state == STATE_WAITING then
        
            -- Read proximity sensor to see if object reached pick target
            local detected, distance, detectedPoint, detectedObject =
                sim.readProximitySensor(pickSensor)
    
            if detected == 1 then
                print('Object detected!')
                
                -- Stop conveyor once object reaches pick zone
                sim.writeCustomTableData(conveyorBelt, '__ctrl__', {vel = 0.0, accel = 0.0})
                activeObject = detectedObject
                
                -- Disable object dynamics before pickup to avoid physics-kinematics conflicts
                setObjectDynamic(activeObject, false)
                
                state = STATE_PICKING       -- move to next state
                print('-> STATE_PICKING')   -- status message (for debug)
            end
        end
    
        ------------------------------------------------------------------
        -- PICK OBJECT
        ------------------------------------------------------------------
        if state == STATE_PICKING then
        
            -- Move robot with IK and pick object
            local picked = pickObject(pickTarget, activeObject, false)
    
            if picked then
                state = STATE_PLACING       -- move to next state
                print('-> STATE_PLACING')   -- status message (for debug)
            end
        end

        ------------------------------------------------------------------
        -- PLACE OBJECT
        ------------------------------------------------------------------
        if state == STATE_PLACING then
        
            local placed = false                 -- primary placement status
            local placedOnShelf = false          -- shelf placement status
            local placeTarget                    -- placement target
            local objectType = item[objIndex]    -- object type (cuboid, cylinder, sphere)
            
            -- Decide primary target
            if objectType == cuboid then
                placeTarget = targetShelves[shelfIndex]
                print('TASK: Place object to SHELF target', shelfIndex)
            elseif objectType == cylinder then
                placeTarget = targetExit
                print('TASK: Place object to EXIT CONVEYOR')
            else -- sphere or anything else
                placeTarget = targetBin
                print('TASK: Place object to BIN')
            end
        
            -- Try placing object to primary target
            placed = placeObject(placeTarget, activeObject, true)
            
            if placed and objectType == cuboid then
                placedOnShelf = true
            end
            
            -- Make another attempt if primary target fails
            if not placed then
                print('FAIL: Retrying placement once...')
                -- Move robot back to home position and try again
                moveToHome(false)
                placed = placeObject(placeTarget, activeObject, true)
                if placed and objectType == cuboid then
                    placedOnShelf = true
                end
            end
        
            -- If it fails again, then target may not be reachable
            -- Try to place item at fallback target (exit conveyor)
            if not placed then
                print('TASK FAIL: Trying fallback target (EXIT)')
                placed = placeObject(targetExit, activeObject, true)
                placedOnShelf = false   -- item was not placed on shelf even if it's a cuboid
            end
        
            -- If placement succeeded
            if placed then
            
                -- Re-enable object dynamics for dropping and conveyor interaction 
                setObjectDynamic(activeObject, true)
        
                -- Only register shelf objects if actually placed on shelf
                if placedOnShelf then                
                    shelfObjects[#shelfObjects + 1] = activeObject   -- store object handle
                    shelfIndex = shelfIndex + 1                      -- increment index
                end               
                
                -- Cleanup shelves if full
                if shelfIndex > #targetShelves then
                    -- First make sure all objects added to shelfObjects list are valid
                    local valid = {}   -- make new list
                    for i = 1, #shelfObjects do
                        if sim.isHandle(shelfObjects[i]) then
                            valid[#valid + 1] = shelfObjects[i]
                        end
                    end
                    -- If at least one valid object is on the shelves (rack) then remove
                    if #valid > 0 then
                        sim.removeObjects(valid, false)
                    end
                    shelfObjects = {}   -- clear list
                    shelfIndex = 1      -- reset index
                end
                
            -- If it still fails, then drop obejct, move to home, and repeat cycle
            else
                print('ERROR: Primary and fallback placement failed!')
                       
                sim.setObjectParent(activeObject, -1, true) -- ensure object is dropped
                setObjectDynamic(activeObject, true)        -- re-enable object dynamics
            end
            
            activeObject = nil           -- clear flag
            sim.wait(0.1)                -- small delay for any pending tasks to run
            moveToHome(false)            -- Return robot to home position
            state = STATE_SPAWNING       -- move to next state
            print('-> STATE_SPAWNING')   -- status message (for debug)
        end

        ------------------------------------------------------------------
        -- SPAWN NEXT OBJECT AND RESTART
        ------------------------------------------------------------------
        if state == STATE_SPAWNING then
            -- Cycle object index through item list without exceeding bounds
            objIndex= (objIndex% #item) + 1
           
            -- Spawn next object from the item list (or restart the list)
            activeObject = spawnObjectDefault(item[objIndex], spawnTarget)
            if activeObject then
                print('Spawned object')
            end
    
            -- Restart conveyor
            sim.writeCustomTableData(conveyorBelt, '__ctrl__', {vel = 0.6})
                
            -- Clear previous path drawing
            sim.addDrawingObjectItem(drawingHandle, nil)
    
            state = STATE_WAITING
            print('-> STATE_WAITING')   -- status message (for debug)
        end
        ------------------------------------------------------------------
        sim.wait(0.01) -- small delay for stability
        
    end -- while
end
