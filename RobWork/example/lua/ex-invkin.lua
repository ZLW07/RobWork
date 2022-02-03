require("sdurw_core")
require("sdurw_math")
require("sdurw_kinematics")
require("sdurw_models")
require("sdurw")

using("sdurw_math")
using("sdurw_kinematics")
using("sdurw_models")
using("sdurw")

if #arg ~= 1 then
    print("Usage: lua ex-invkin.lua <path/to/RobWorkData>")
    return 1
end

local WC_FILE = "/devices/serialdev/UR10e_2018/UR10e.xml"

local wc = WorkCellLoaderFactory.load(arg[1] .. WC_FILE)
if wc:isNull() then
    error("WorkCell could not be loaded")
end
local device = wc:findSerialDevice("UR10e")
if device:isNull() then
    error("UR10e device could not be found.")
end

local state = wc:getDefaultState()
local solver = ClosedFormIKSolverUR(device:cptr(), state)

local Tdesired = Transform3Dd(Vector3Dd(0.2, -0.2, 0.5), EAAd(0, Pi, 0):toRotation3D())
local solutions = solver:solve(Tdesired, state)

print("Inverse Kinematics for " .. device:getName() .. ".")
print(" Base frame: " .. device:getBase():getName())
print(" End/TCP frame: " .. solver:getTCP():getName())
print(" Target Transform: " .. tostring(Tdesired))
print("Found " .. solutions:size() .. " solutions.")
for i= 0,solutions:size()-1,1
do
    print(" " .. tostring(solutions[i]))
end
