--
-- scene should have two bodies one fixed (FBody) and one RigidBody (RBody) that are constrained by a motorized joint
-- 

-- first load robwork lua interface
rwroot = getenv("RW_ROOT")
robwork_init=loadlib(rwroot .. "/libs/release/sdurwlua.so","luaopen_sdurw")
assert(robwork_init)
robworkstudio_init=loadlib(rwroot .. "../RobWorkStudio/libs/release/sdurwslua.so","luaopen_sdurws")
assert(robworkstudio_init)

robwork_init();
robworkstudio_init();

-- next 




