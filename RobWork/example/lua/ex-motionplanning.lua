require("sdurw_core")
require("sdurw_models")
require("sdurw_proximity")
require("sdurw")
require("sdurw_pathplanners")
require("sdurw_proximitystrategies")
using("sdurw_models")
using("sdurw_proximity")
using("sdurw")
using("sdurw_pathplanners")
using("sdurw_proximitystrategies")

require("sdurw_math")
using("sdurw_math")

if #arg ~= 1 then
    print("Usage: lua ex-motionplanning.lua <path/to/RobWorkData>")
    return 1
end

local WC_FILE = "/scenes/SinglePA10Demo/SinglePA10DemoGantry.wc.xml"

local wc = WorkCellLoaderFactory.load(arg[1] .. WC_FILE)
if wc:isNull() then
    error("WorkCell could not be loaded")
end
local gantry = wc:findDevice("Gantry")
local pa10 = wc:findDevice("PA10")
if gantry:isNull() then
    error("Gantry device could not be found.")
end
if pa10:isNull() then
    error("PA10 device could not be found.")
end

local state = wc:getDefaultState()
local device = sdurw_models.ownedPtr(CompositeDevice(gantry:getBase(), wc:getDevices(),
                        pa10:getEnd(), "Composite", state))

local cdstrategy = sdurw_proximitystrategies.ProximityStrategyFactory.makeCollisionStrategy("PQP")
if cdstrategy:isNull() then
    error("PQP Collision Strategy could not be found.")
end
local collisionDetector = sdurw_proximity.ownedPtr(CollisionDetector(wc, cdstrategy))
local con = PlannerConstraint.make(collisionDetector, device:asDeviceCPtr(), state)
local planner = RRTPlanner.makeQToQPlanner(con, device:asDevicePtr())

local beg = Q(9, -0.67, -0.79, 2.8, -0.02, -1.01, -0.26, -0.77, -1.01, 0)
local fin = Q(9, 0.57, 0.79, -1.23, 0.21, -0.63, -0.55, -0.07, -1.08, 0)

local pdata = ProximityData()
device:setQ(beg, state)
if collisionDetector:inCollision(state, pdata) then
    error("Initial configuration in collision!")
end
device:setQ(fin, state)
if collisionDetector:inCollision(state, pdata) then
    error("Final configuration in collision!")
end

local result = PathQ()
if planner:query(beg, fin, result) then
    print("Planned path with " .. result:size() .. " configurations")
end
