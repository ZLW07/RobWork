-- EXAMPLE HEADER STARTS HERE
require("sdurw")
require("sdurws")

function openpackage (ns)
for n,v in pairs(ns) do
    if _G[n] ~= nil then
        print("name clash: " .. n .. " is already defined")
        else
        _G[n] = v
        end
    end
end

openpackage(sdurw)
openpackage(sdurws)

-- helper function - adds a configuration to the state path
function addQ(tpath, time, arr)
 arm:setQ( sdurw.Q(#arr,arr), state) -- set the device configuration in state
 tpath:add( time, state)
end

-- helper function - add a grasp to the state function
function graspObject(tpath, obj, tool)
 sdurw.gripFrame(state, obj, tool)
 time = tpath[tpath:size()-1]:getTime()
 tpath:add(time, state)
end

rwstudio = getRobWorkStudioInstance()

-- load workcell
wc = rwstudio:getWorkCell()
state = wc:getDefaultState() -- the state

-- EXAMPLE BODY STARTS HERE

-- load device, gripper, object and state
arm = wc:findDevice("PA10") -- robot arm
gripper = wc:findFrame("Tool") -- the end effector frame which is the gripper
object = wc:findFrame("Bottle") -- the object which is to be grasped
table = wc:findFrame("Table") -- the object which is to be grasped

-- we store everything in a timed state path
tpath = sdurw.PathTimedState()
addQ( tpath, 1, {0,0.1,0,0,0,0,0} )
addQ( tpath, 1, {0,0.1,0,0,0,0,0} )
addQ( tpath, 2, {0,0.2,0,0,0,0,0} )
addQ( tpath, 3, {0,0.3,0,0,0,0,0} )
addQ( tpath, 4, {0,0.4,0,0,0,0,0} )
addQ( tpath, 5, {0,0.5,0,0,0,0,0} )
addQ( tpath, 6, {0,0.6,0,0,0,0,0} )
addQ( tpath, 7, {0,0.7,0,0,0,0,0} )
-- grasp the object
graspObject( tpath, object, gripper)
-- samples are int the form add(time, Q)
addQ( tpath,  8, {0,0.7,0,0,0,0,0} )
addQ( tpath,  9, {0,0.6,0,0,0,0,0} )
addQ( tpath,  10, {0,0.5,0,0,0,0,0} )
addQ( tpath,  11, {0,0.4,0,0,0,0,0} )
addQ( tpath,  12, {0,0.3,0,0,0,0,0} )
addQ( tpath,  13, {0,0.2,0,0,0,0,0} )
addQ( tpath, 14, {0,0.1,0,0,0,0,0} )

graspObject( tpath, object, table)

addQ( tpath, 15, {0,0.1,0,0,0,0,0} )
addQ( tpath, 16, {0,0.1,0,0,0,0,0} )
addQ( tpath, 18, {0,0.2,0,0,0,0,0} )
addQ( tpath, 19, {0,0.3,0,0,0,0,0} )

rwstudio:setTimedStatePath( tpath )