function findFromWorkCell(wc)
    -- get the default state
    local state = wc:getDefaultState();
    local worldFrame = wc:getWorldFrame();
    -- find a frame by name, remember NULL is a valid return
    local frame = wc:findFrame("FixedFrameName");
    -- find a frame by name, but with a specific frame type
    local fframe = wc:findFixedFrame("FixedFrameName");
    local mframe = wc:findMovableFrame("MovableFrameName");
    -- find a device by name
    local device = wc:findDevice("SerialDeviceName");
    local sdevice = wc:findSerialDevice("SerialDeviceName");
end
