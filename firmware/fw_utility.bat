@echo off

if "%1"=="compile" (
    arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen  controller_fw 
    @REM arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8  controller_fw 
) else if "%1"=="upload" (
    arduino-cli upload -p COM6 --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen controller_fw 
)  else if "%1"=="all" (
    arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen  controller_fw  
    arduino-cli upload --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen controller_fw 
)  else (
    echo invalid argument
)

exit
