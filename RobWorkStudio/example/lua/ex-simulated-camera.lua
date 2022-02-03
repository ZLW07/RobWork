require("sdurw_core")
require("sdurw_kinematics")
require("sdurw_models")
require("sdurw_sensor")
require("sdurw")
require("sdurw_simulation")
require("sdurws")

using("sdurw_core")
using("sdurw_kinematics")
using("sdurw")
using("sdurw_simulation")
using("sdurws")

if #arg < 1 then
    error("Provide the path to RobWorkData as first argument.")
end

local WC_FILE = arg[1] .. "/scenes/SensorTestScene/SimpleWorkcell.xml"
print(WC_FILE)

local wc = WorkCellLoaderFactory.load(WC_FILE)
if wc:isNull() then
    error("WorkCell could not be loaded")
end
local camera = wc:findFrame("Camera")
if camera == nil then
    error("Camera frame could not be found.")
end
local properties = camera:getPropertyMap();
if not properties:has("Camera") then
    error("Camera frame does not have Camera property.")
end
local parameters = properties:getString("Camera");
local parlist={}
for str in string.gmatch(parameters, "([^%s]+)") do
    table.insert(parlist, str)
end
fovy = tonumber(parlist[1])
width = tonumber(parlist[2])
height = tonumber(parlist[3])
print("Camera properties: fov " .. tostring(fovy) .. " width " .. tostring(width) .. " height " .. tostring(height));

local rwstudio = getRobWorkStudioInstance();
rwstudio:postOpenWorkCell(WC_FILE);
sleep(2);
local gldrawer = rwstudio:getView():getSceneViewer();
local framegrabber = ownedPtr( GLFrameGrabber(width,height,fovy) );
framegrabber:init(gldrawer);
local simcam = SimulatedCamera("SimulatedCamera", fovy, camera, framegrabber:asFrameGrabberPtr());
simcam:setFrameRate(100);
simcam:initialize();
simcam:start();
simcam:acquire();

local DT = 0.001;
local info = UpdateInfo(DT);
local state = wc:getDefaultState();
local cnt = 0;
local img;
while not simcam:isImageReady() do
    print("Image is not ready yet. Iteration " .. tostring(cnt));
    simcam:update(info, state);
    cnt = cnt+1;
end
img = simcam:getImage();
img:saveAsPPM("Image1.ppm");
simcam:acquire();
while not simcam:isImageReady() do
    print("Image is not ready yet. Iteration " .. tostring(cnt));
    simcam:update(info, state);
    cnt = cnt+1;
end
print("Took " .. tostring(cnt) .. " steps");
img = simcam:getImage();
print("Image: " .. tostring(img:getWidth()) .. "x" .. tostring(img:getHeight()) .. " bits " .. tostring(img:getBitsPerPixel()) .. " channels " .. tostring(img:getNrOfChannels()));
img:saveAsPPM("Image2.ppm");

simcam:stop();
sleep(20);
rwstudio:postExit();
sleep(1);
