function fwdKinematics(device, frame, mframe, state)
    -- calculate the transform from one frame to another
    fTmf = Kinematics.frameTframe(frame, mframe, state);
    -- calculate the transform from world to frame
    wTmf = Kinematics.worldTframe( mframe, state );
    -- we can find the world to frame transform by a little jogling
    wTf = wTmf * inverse(fTmf);
    -- test if frame is a dynamic attachable frame
    if Kinematics.isDAF( mframe ) then
       -- attach mframe to end of serial device
       Kinematics.gripFrame(mframe, sdevice:getEnd(), state);
    end
end