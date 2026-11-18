-- ap : launcher. Usage: ap <command> ...   (see `ap` with no arguments)
package.path = package.path .. ";/ap/lib/?.lua;/ap/?.lua"
local f = assert(loadfile("/ap/main.lua", "t", _ENV))
return f(...)
