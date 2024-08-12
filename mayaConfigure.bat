setlocal

SET MAYA_VERSION=2024
SET BUILD=mayabuild_%MAYA_VERSION%
SET COMPILER=Visual Studio 16 2019

cmake -B ./%BUILD% -DMAYA_VERSION="%MAYA_VERSION%" -G "%COMPILER%"
cmake --build ./%BUILD% --config RelWithDebInfo

pause
