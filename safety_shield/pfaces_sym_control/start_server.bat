@echo off

rem Prompt the user to confirm the IP address
set HOST_IP=127.0.0.1
set /p HOST_IP="Provide Host IP Address (Press ENTER to use the default="%HOST_IP%"): "
echo pFaces-SymControl server will be using the following Host IP: %HOST_IP%
echo You can check it from any brouser by visiting: http://%HOST_IP%:12345/pFaces/REST/dictionary/morai_acas
pause

set SYM_CONTROL_KERNEL=..\..\..\pFaces\tests\samples\pFaces-SymbolicControl\kernel-pack
pfaces -CG -v3 -d 1 -k gb_fp@%SYM_CONTROL_KERNEL% -cfg .\morai_acas.cfg -co "online_mode=true" -co "online_mode_host=%HOST_IP%" -co "online_mode_port=12345"