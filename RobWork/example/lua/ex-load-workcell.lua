require("sdurw_core")
require("sdurw_models")
require("sdurw")
using("sdurw")

if #arg ~= 1 then
    print("Usage: lua ex-load-workcell.lua <workcell>")
    return 1
end

workcell = WorkCellLoaderFactory.load(arg[1])
if workcell:isNull() then
    print("WorkCell could not be loaded")
    return 1
end

print("Workcell " .. workcell:getName() .. " successfully loaded.");