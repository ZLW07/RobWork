function printDeviceNames(workcell)
    print("Workcell " .. workcell:getName() .. " contains devices:");
    devices = workcell:getDevices() 
    for i = 0,devices:size()-1 do
        print("- " .. devices[i]:getName())
    end
end
