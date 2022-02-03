require("sdurw_core")
require("sdurw")
using("sdurw")
require("sdurw_math")
using("sdurw_math")

rotd = Rotation3Dd(1,0,0,0,0,-1,0,1,0);
rotf = Rotation3Df(1,0,0,0,0,-1,0,1,0);

print("Rotation double:");
print(tostring(rotd));
print("Rotation float:");
print(tostring(rotf));
print("Rotation inverse:");
print(tostring(inverse(rotd)));
print("Identity:");
print(tostring(rotd*inverse(rotd)));