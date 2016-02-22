################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/subdir.mk
-include Graphics/subdir.mk
-include Drivers/subdir.mk
-include CMSIS/efm32hg/subdir.mk
-include BSP/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: SLSTK3400A_humitemp.axf

# Tool invocations
SLSTK3400A_humitemp.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m0plus -mthumb -T "SLSTK3400A_humitemp.ld" -Xlinker --gc-sections -Xlinker -Map="SLSTK3400A_humitemp.map" --specs=nano.specs -o SLSTK3400A_humitemp.axf "./src/graphics.o" "./src/humitemp.o" "./emlib/em_adc.o" "./emlib/em_assert.o" "./emlib/em_cmu.o" "./emlib/em_emu.o" "./emlib/em_gpio.o" "./emlib/em_i2c.o" "./emlib/em_int.o" "./emlib/em_leuart.o" "./emlib/em_rtc.o" "./emlib/em_usart.o" "./Graphics/dmd_display.o" "./Graphics/glib.o" "./Graphics/glib_bitmap.o" "./Graphics/glib_circle.o" "./Graphics/glib_font_narrow_6x8.o" "./Graphics/glib_font_normal_8x8.o" "./Graphics/glib_line.o" "./Graphics/glib_rectangle.o" "./Graphics/glib_string.o" "./Drivers/display.o" "./Drivers/displayls013b7dh03.o" "./Drivers/displaypalemlib.o" "./Drivers/i2cspm.o" "./Drivers/rtcdriver.o" "./Drivers/si7013.o" "./Drivers/udelay.o" "./CMSIS/efm32hg/startup_efm32hg.o" "./CMSIS/efm32hg/system_efm32hg.o" "./BSP/bsp_bcc.o" "./BSP/bsp_stk.o" "./BSP/bsp_stk_leds.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: SLSTK3400A_humitemp.hex'
	arm-none-eabi-objcopy -O ihex "SLSTK3400A_humitemp.axf" "SLSTK3400A_humitemp.hex"
	@echo ' '

	@echo 'Building bin file: SLSTK3400A_humitemp.bin'
	arm-none-eabi-objcopy -O binary "SLSTK3400A_humitemp.axf" "SLSTK3400A_humitemp.bin"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "SLSTK3400A_humitemp.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EXECUTABLES) SLSTK3400A_humitemp.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
