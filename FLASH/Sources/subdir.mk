################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.local

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS_QUOTED += \
"../Sources/CANdrv.c" \
"../Sources/UART_drv.c" \
"../Sources/adc_drv.c" \
"../Sources/adc_init.c" \
"../Sources/adc_trigger_init.c" \
"../Sources/common.c" \
"../Sources/core_ai.c" \
"../Sources/dc_motor.c" \
"../Sources/draw_in_glcd.c" \
"../Sources/dspi_init.c" \
"../Sources/dspi_var.c" \
"../Sources/emios_init.c" \
"../Sources/encoder.c" \
"../Sources/external_interrupt.c" \
"../Sources/flexcan_init.c" \
"../Sources/freemaster_MPC56xx.c" \
"../Sources/freemaster_appcmd.c" \
"../Sources/freemaster_bdm.c" \
"../Sources/freemaster_protocol.c" \
"../Sources/freemaster_rec.c" \
"../Sources/freemaster_scope.c" \
"../Sources/freemaster_serial.c" \
"../Sources/freemaster_sfio.c" \
"../Sources/freemaster_tsa.c" \
"../Sources/gpio_drv.c" \
"../Sources/intc_init.c" \
"../Sources/intc_pit.c" \
"../Sources/intc_sw_vecttable.c" \
"../Sources/line_scan.c" \
"../Sources/linflex_init.c" \
"../Sources/linflex_var.c" \
"../Sources/main.c" \
"../Sources/photo_sensor_hld.c" \
"../Sources/pit_init.c" \
"../Sources/pot_hld.c" \
"../Sources/rappid_utils.c" \
"../Sources/rchw_init.c" \
"../Sources/romcopy.c" \
"../Sources/rtc_init.c" \
"../Sources/sbc_hld.c" \
"../Sources/school_zone_adc.c" \
"../Sources/servo_motor.c" \
"../Sources/siu_init.c" \
"../Sources/slope.c" \
"../Sources/sona_sensor.c" \
"../Sources/st7565.c" \
"../Sources/stm_init.c" \
"../Sources/swt_init.c" \
"../Sources/sys_init.c" \
"../Sources/sysclk_init.c" \

C_SRCS += \
../Sources/CANdrv.c \
../Sources/UART_drv.c \
../Sources/adc_drv.c \
../Sources/adc_init.c \
../Sources/adc_trigger_init.c \
../Sources/common.c \
../Sources/core_ai.c \
../Sources/dc_motor.c \
../Sources/draw_in_glcd.c \
../Sources/dspi_init.c \
../Sources/dspi_var.c \
../Sources/emios_init.c \
../Sources/encoder.c \
../Sources/external_interrupt.c \
../Sources/flexcan_init.c \
../Sources/freemaster_MPC56xx.c \
../Sources/freemaster_appcmd.c \
../Sources/freemaster_bdm.c \
../Sources/freemaster_protocol.c \
../Sources/freemaster_rec.c \
../Sources/freemaster_scope.c \
../Sources/freemaster_serial.c \
../Sources/freemaster_sfio.c \
../Sources/freemaster_tsa.c \
../Sources/gpio_drv.c \
../Sources/intc_init.c \
../Sources/intc_pit.c \
../Sources/intc_sw_vecttable.c \
../Sources/line_scan.c \
../Sources/linflex_init.c \
../Sources/linflex_var.c \
../Sources/main.c \
../Sources/photo_sensor_hld.c \
../Sources/pit_init.c \
../Sources/pot_hld.c \
../Sources/rappid_utils.c \
../Sources/rchw_init.c \
../Sources/romcopy.c \
../Sources/rtc_init.c \
../Sources/sbc_hld.c \
../Sources/school_zone_adc.c \
../Sources/servo_motor.c \
../Sources/siu_init.c \
../Sources/slope.c \
../Sources/sona_sensor.c \
../Sources/st7565.c \
../Sources/stm_init.c \
../Sources/swt_init.c \
../Sources/sys_init.c \
../Sources/sysclk_init.c \

S_SRCS += \
../Sources/crt0.s \
../Sources/excep_handler.s \
../Sources/rappid_func.s \
../Sources/sw_handlers.s \

S_SRCS_QUOTED += \
"../Sources/crt0.s" \
"../Sources/excep_handler.s" \
"../Sources/rappid_func.s" \
"../Sources/sw_handlers.s" \

S_DEPS_QUOTED += \
"./Sources/crt0_s.d" \
"./Sources/excep_handler_s.d" \
"./Sources/rappid_func_s.d" \
"./Sources/sw_handlers_s.d" \

OBJS += \
./Sources/CANdrv_c.obj \
./Sources/UART_drv_c.obj \
./Sources/adc_drv_c.obj \
./Sources/adc_init_c.obj \
./Sources/adc_trigger_init_c.obj \
./Sources/common_c.obj \
./Sources/core_ai_c.obj \
./Sources/crt0_s.obj \
./Sources/dc_motor_c.obj \
./Sources/draw_in_glcd_c.obj \
./Sources/dspi_init_c.obj \
./Sources/dspi_var_c.obj \
./Sources/emios_init_c.obj \
./Sources/encoder_c.obj \
./Sources/excep_handler_s.obj \
./Sources/external_interrupt_c.obj \
./Sources/flexcan_init_c.obj \
./Sources/freemaster_MPC56xx_c.obj \
./Sources/freemaster_appcmd_c.obj \
./Sources/freemaster_bdm_c.obj \
./Sources/freemaster_protocol_c.obj \
./Sources/freemaster_rec_c.obj \
./Sources/freemaster_scope_c.obj \
./Sources/freemaster_serial_c.obj \
./Sources/freemaster_sfio_c.obj \
./Sources/freemaster_tsa_c.obj \
./Sources/gpio_drv_c.obj \
./Sources/intc_init_c.obj \
./Sources/intc_pit_c.obj \
./Sources/intc_sw_vecttable_c.obj \
./Sources/line_scan_c.obj \
./Sources/linflex_init_c.obj \
./Sources/linflex_var_c.obj \
./Sources/main_c.obj \
./Sources/photo_sensor_hld_c.obj \
./Sources/pit_init_c.obj \
./Sources/pot_hld_c.obj \
./Sources/rappid_func_s.obj \
./Sources/rappid_utils_c.obj \
./Sources/rchw_init_c.obj \
./Sources/romcopy_c.obj \
./Sources/rtc_init_c.obj \
./Sources/sbc_hld_c.obj \
./Sources/school_zone_adc_c.obj \
./Sources/servo_motor_c.obj \
./Sources/siu_init_c.obj \
./Sources/slope_c.obj \
./Sources/sona_sensor_c.obj \
./Sources/st7565_c.obj \
./Sources/stm_init_c.obj \
./Sources/sw_handlers_s.obj \
./Sources/swt_init_c.obj \
./Sources/sys_init_c.obj \
./Sources/sysclk_init_c.obj \

S_DEPS += \
./Sources/crt0_s.d \
./Sources/excep_handler_s.d \
./Sources/rappid_func_s.d \
./Sources/sw_handlers_s.d \

OBJS_QUOTED += \
"./Sources/CANdrv_c.obj" \
"./Sources/UART_drv_c.obj" \
"./Sources/adc_drv_c.obj" \
"./Sources/adc_init_c.obj" \
"./Sources/adc_trigger_init_c.obj" \
"./Sources/common_c.obj" \
"./Sources/core_ai_c.obj" \
"./Sources/crt0_s.obj" \
"./Sources/dc_motor_c.obj" \
"./Sources/draw_in_glcd_c.obj" \
"./Sources/dspi_init_c.obj" \
"./Sources/dspi_var_c.obj" \
"./Sources/emios_init_c.obj" \
"./Sources/encoder_c.obj" \
"./Sources/excep_handler_s.obj" \
"./Sources/external_interrupt_c.obj" \
"./Sources/flexcan_init_c.obj" \
"./Sources/freemaster_MPC56xx_c.obj" \
"./Sources/freemaster_appcmd_c.obj" \
"./Sources/freemaster_bdm_c.obj" \
"./Sources/freemaster_protocol_c.obj" \
"./Sources/freemaster_rec_c.obj" \
"./Sources/freemaster_scope_c.obj" \
"./Sources/freemaster_serial_c.obj" \
"./Sources/freemaster_sfio_c.obj" \
"./Sources/freemaster_tsa_c.obj" \
"./Sources/gpio_drv_c.obj" \
"./Sources/intc_init_c.obj" \
"./Sources/intc_pit_c.obj" \
"./Sources/intc_sw_vecttable_c.obj" \
"./Sources/line_scan_c.obj" \
"./Sources/linflex_init_c.obj" \
"./Sources/linflex_var_c.obj" \
"./Sources/main_c.obj" \
"./Sources/photo_sensor_hld_c.obj" \
"./Sources/pit_init_c.obj" \
"./Sources/pot_hld_c.obj" \
"./Sources/rappid_func_s.obj" \
"./Sources/rappid_utils_c.obj" \
"./Sources/rchw_init_c.obj" \
"./Sources/romcopy_c.obj" \
"./Sources/rtc_init_c.obj" \
"./Sources/sbc_hld_c.obj" \
"./Sources/school_zone_adc_c.obj" \
"./Sources/servo_motor_c.obj" \
"./Sources/siu_init_c.obj" \
"./Sources/slope_c.obj" \
"./Sources/sona_sensor_c.obj" \
"./Sources/st7565_c.obj" \
"./Sources/stm_init_c.obj" \
"./Sources/sw_handlers_s.obj" \
"./Sources/swt_init_c.obj" \
"./Sources/sys_init_c.obj" \
"./Sources/sysclk_init_c.obj" \

C_DEPS += \
./Sources/CANdrv_c.d \
./Sources/UART_drv_c.d \
./Sources/adc_drv_c.d \
./Sources/adc_init_c.d \
./Sources/adc_trigger_init_c.d \
./Sources/common_c.d \
./Sources/core_ai_c.d \
./Sources/dc_motor_c.d \
./Sources/draw_in_glcd_c.d \
./Sources/dspi_init_c.d \
./Sources/dspi_var_c.d \
./Sources/emios_init_c.d \
./Sources/encoder_c.d \
./Sources/external_interrupt_c.d \
./Sources/flexcan_init_c.d \
./Sources/freemaster_MPC56xx_c.d \
./Sources/freemaster_appcmd_c.d \
./Sources/freemaster_bdm_c.d \
./Sources/freemaster_protocol_c.d \
./Sources/freemaster_rec_c.d \
./Sources/freemaster_scope_c.d \
./Sources/freemaster_serial_c.d \
./Sources/freemaster_sfio_c.d \
./Sources/freemaster_tsa_c.d \
./Sources/gpio_drv_c.d \
./Sources/intc_init_c.d \
./Sources/intc_pit_c.d \
./Sources/intc_sw_vecttable_c.d \
./Sources/line_scan_c.d \
./Sources/linflex_init_c.d \
./Sources/linflex_var_c.d \
./Sources/main_c.d \
./Sources/photo_sensor_hld_c.d \
./Sources/pit_init_c.d \
./Sources/pot_hld_c.d \
./Sources/rappid_utils_c.d \
./Sources/rchw_init_c.d \
./Sources/romcopy_c.d \
./Sources/rtc_init_c.d \
./Sources/sbc_hld_c.d \
./Sources/school_zone_adc_c.d \
./Sources/servo_motor_c.d \
./Sources/siu_init_c.d \
./Sources/slope_c.d \
./Sources/sona_sensor_c.d \
./Sources/st7565_c.d \
./Sources/stm_init_c.d \
./Sources/swt_init_c.d \
./Sources/sys_init_c.d \
./Sources/sysclk_init_c.d \

OBJS_OS_FORMAT += \
./Sources/CANdrv_c.obj \
./Sources/UART_drv_c.obj \
./Sources/adc_drv_c.obj \
./Sources/adc_init_c.obj \
./Sources/adc_trigger_init_c.obj \
./Sources/common_c.obj \
./Sources/core_ai_c.obj \
./Sources/crt0_s.obj \
./Sources/dc_motor_c.obj \
./Sources/draw_in_glcd_c.obj \
./Sources/dspi_init_c.obj \
./Sources/dspi_var_c.obj \
./Sources/emios_init_c.obj \
./Sources/encoder_c.obj \
./Sources/excep_handler_s.obj \
./Sources/external_interrupt_c.obj \
./Sources/flexcan_init_c.obj \
./Sources/freemaster_MPC56xx_c.obj \
./Sources/freemaster_appcmd_c.obj \
./Sources/freemaster_bdm_c.obj \
./Sources/freemaster_protocol_c.obj \
./Sources/freemaster_rec_c.obj \
./Sources/freemaster_scope_c.obj \
./Sources/freemaster_serial_c.obj \
./Sources/freemaster_sfio_c.obj \
./Sources/freemaster_tsa_c.obj \
./Sources/gpio_drv_c.obj \
./Sources/intc_init_c.obj \
./Sources/intc_pit_c.obj \
./Sources/intc_sw_vecttable_c.obj \
./Sources/line_scan_c.obj \
./Sources/linflex_init_c.obj \
./Sources/linflex_var_c.obj \
./Sources/main_c.obj \
./Sources/photo_sensor_hld_c.obj \
./Sources/pit_init_c.obj \
./Sources/pot_hld_c.obj \
./Sources/rappid_func_s.obj \
./Sources/rappid_utils_c.obj \
./Sources/rchw_init_c.obj \
./Sources/romcopy_c.obj \
./Sources/rtc_init_c.obj \
./Sources/sbc_hld_c.obj \
./Sources/school_zone_adc_c.obj \
./Sources/servo_motor_c.obj \
./Sources/siu_init_c.obj \
./Sources/slope_c.obj \
./Sources/sona_sensor_c.obj \
./Sources/st7565_c.obj \
./Sources/stm_init_c.obj \
./Sources/sw_handlers_s.obj \
./Sources/swt_init_c.obj \
./Sources/sys_init_c.obj \
./Sources/sysclk_init_c.obj \

C_DEPS_QUOTED += \
"./Sources/CANdrv_c.d" \
"./Sources/UART_drv_c.d" \
"./Sources/adc_drv_c.d" \
"./Sources/adc_init_c.d" \
"./Sources/adc_trigger_init_c.d" \
"./Sources/common_c.d" \
"./Sources/core_ai_c.d" \
"./Sources/dc_motor_c.d" \
"./Sources/draw_in_glcd_c.d" \
"./Sources/dspi_init_c.d" \
"./Sources/dspi_var_c.d" \
"./Sources/emios_init_c.d" \
"./Sources/encoder_c.d" \
"./Sources/external_interrupt_c.d" \
"./Sources/flexcan_init_c.d" \
"./Sources/freemaster_MPC56xx_c.d" \
"./Sources/freemaster_appcmd_c.d" \
"./Sources/freemaster_bdm_c.d" \
"./Sources/freemaster_protocol_c.d" \
"./Sources/freemaster_rec_c.d" \
"./Sources/freemaster_scope_c.d" \
"./Sources/freemaster_serial_c.d" \
"./Sources/freemaster_sfio_c.d" \
"./Sources/freemaster_tsa_c.d" \
"./Sources/gpio_drv_c.d" \
"./Sources/intc_init_c.d" \
"./Sources/intc_pit_c.d" \
"./Sources/intc_sw_vecttable_c.d" \
"./Sources/line_scan_c.d" \
"./Sources/linflex_init_c.d" \
"./Sources/linflex_var_c.d" \
"./Sources/main_c.d" \
"./Sources/photo_sensor_hld_c.d" \
"./Sources/pit_init_c.d" \
"./Sources/pot_hld_c.d" \
"./Sources/rappid_utils_c.d" \
"./Sources/rchw_init_c.d" \
"./Sources/romcopy_c.d" \
"./Sources/rtc_init_c.d" \
"./Sources/sbc_hld_c.d" \
"./Sources/school_zone_adc_c.d" \
"./Sources/servo_motor_c.d" \
"./Sources/siu_init_c.d" \
"./Sources/slope_c.d" \
"./Sources/sona_sensor_c.d" \
"./Sources/st7565_c.d" \
"./Sources/stm_init_c.d" \
"./Sources/swt_init_c.d" \
"./Sources/sys_init_c.d" \
"./Sources/sysclk_init_c.d" \


# Each subdirectory must supply rules for building sources it contributes
Sources/CANdrv_c.obj: ../Sources/CANdrv.c
	@echo 'Building file: $<'
	@echo 'Executing target #1 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/CANdrv.args" -o "Sources/CANdrv_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.c
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/UART_drv_c.obj: ../Sources/UART_drv.c
	@echo 'Building file: $<'
	@echo 'Executing target #2 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/UART_drv.args" -o "Sources/UART_drv_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/adc_drv_c.obj: ../Sources/adc_drv.c
	@echo 'Building file: $<'
	@echo 'Executing target #3 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/adc_drv.args" -o "Sources/adc_drv_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/adc_init_c.obj: ../Sources/adc_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #4 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/adc_init.args" -o "Sources/adc_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/adc_trigger_init_c.obj: ../Sources/adc_trigger_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #5 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/adc_trigger_init.args" -o "Sources/adc_trigger_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/common_c.obj: ../Sources/common.c
	@echo 'Building file: $<'
	@echo 'Executing target #6 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/common.args" -o "Sources/common_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/core_ai_c.obj: ../Sources/core_ai.c
	@echo 'Building file: $<'
	@echo 'Executing target #7 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/core_ai.args" -o "Sources/core_ai_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/crt0_s.obj: ../Sources/crt0.s
	@echo 'Building file: $<'
	@echo 'Executing target #8 $<'
	@echo 'Invoking: PowerPC Assembler'
	"$(PAToolsDirEnv)/mwasmeppc" @@"Sources/crt0.args" -o "Sources/crt0_s.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/%.d: ../Sources/%.s
	@echo 'Regenerating dependency file: $@'
	
	@echo ' '

Sources/dc_motor_c.obj: ../Sources/dc_motor.c
	@echo 'Building file: $<'
	@echo 'Executing target #9 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/dc_motor.args" -o "Sources/dc_motor_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/draw_in_glcd_c.obj: ../Sources/draw_in_glcd.c
	@echo 'Building file: $<'
	@echo 'Executing target #10 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/draw_in_glcd.args" -o "Sources/draw_in_glcd_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/dspi_init_c.obj: ../Sources/dspi_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #11 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/dspi_init.args" -o "Sources/dspi_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/dspi_var_c.obj: ../Sources/dspi_var.c
	@echo 'Building file: $<'
	@echo 'Executing target #12 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/dspi_var.args" -o "Sources/dspi_var_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/emios_init_c.obj: ../Sources/emios_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #13 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/emios_init.args" -o "Sources/emios_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/encoder_c.obj: ../Sources/encoder.c
	@echo 'Building file: $<'
	@echo 'Executing target #14 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/encoder.args" -o "Sources/encoder_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/excep_handler_s.obj: ../Sources/excep_handler.s
	@echo 'Building file: $<'
	@echo 'Executing target #15 $<'
	@echo 'Invoking: PowerPC Assembler'
	"$(PAToolsDirEnv)/mwasmeppc" @@"Sources/excep_handler.args" -o "Sources/excep_handler_s.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/external_interrupt_c.obj: ../Sources/external_interrupt.c
	@echo 'Building file: $<'
	@echo 'Executing target #16 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/external_interrupt.args" -o "Sources/external_interrupt_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/flexcan_init_c.obj: ../Sources/flexcan_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #17 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/flexcan_init.args" -o "Sources/flexcan_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_MPC56xx_c.obj: ../Sources/freemaster_MPC56xx.c
	@echo 'Building file: $<'
	@echo 'Executing target #18 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_MPC56xx.args" -o "Sources/freemaster_MPC56xx_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_appcmd_c.obj: ../Sources/freemaster_appcmd.c
	@echo 'Building file: $<'
	@echo 'Executing target #19 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_appcmd.args" -o "Sources/freemaster_appcmd_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_bdm_c.obj: ../Sources/freemaster_bdm.c
	@echo 'Building file: $<'
	@echo 'Executing target #20 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_bdm.args" -o "Sources/freemaster_bdm_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_protocol_c.obj: ../Sources/freemaster_protocol.c
	@echo 'Building file: $<'
	@echo 'Executing target #21 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_protocol.args" -o "Sources/freemaster_protocol_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_rec_c.obj: ../Sources/freemaster_rec.c
	@echo 'Building file: $<'
	@echo 'Executing target #22 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_rec.args" -o "Sources/freemaster_rec_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_scope_c.obj: ../Sources/freemaster_scope.c
	@echo 'Building file: $<'
	@echo 'Executing target #23 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_scope.args" -o "Sources/freemaster_scope_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_serial_c.obj: ../Sources/freemaster_serial.c
	@echo 'Building file: $<'
	@echo 'Executing target #24 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_serial.args" -o "Sources/freemaster_serial_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_sfio_c.obj: ../Sources/freemaster_sfio.c
	@echo 'Building file: $<'
	@echo 'Executing target #25 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_sfio.args" -o "Sources/freemaster_sfio_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/freemaster_tsa_c.obj: ../Sources/freemaster_tsa.c
	@echo 'Building file: $<'
	@echo 'Executing target #26 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/freemaster_tsa.args" -o "Sources/freemaster_tsa_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/gpio_drv_c.obj: ../Sources/gpio_drv.c
	@echo 'Building file: $<'
	@echo 'Executing target #27 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/gpio_drv.args" -o "Sources/gpio_drv_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/intc_init_c.obj: ../Sources/intc_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #28 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/intc_init.args" -o "Sources/intc_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/intc_pit_c.obj: ../Sources/intc_pit.c
	@echo 'Building file: $<'
	@echo 'Executing target #29 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/intc_pit.args" -o "Sources/intc_pit_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/intc_sw_vecttable_c.obj: ../Sources/intc_sw_vecttable.c
	@echo 'Building file: $<'
	@echo 'Executing target #30 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/intc_sw_vecttable.args" -o "Sources/intc_sw_vecttable_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/line_scan_c.obj: ../Sources/line_scan.c
	@echo 'Building file: $<'
	@echo 'Executing target #31 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/line_scan.args" -o "Sources/line_scan_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/linflex_init_c.obj: ../Sources/linflex_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #32 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/linflex_init.args" -o "Sources/linflex_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/linflex_var_c.obj: ../Sources/linflex_var.c
	@echo 'Building file: $<'
	@echo 'Executing target #33 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/linflex_var.args" -o "Sources/linflex_var_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/main_c.obj: ../Sources/main.c
	@echo 'Building file: $<'
	@echo 'Executing target #34 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/main.args" -o "Sources/main_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/photo_sensor_hld_c.obj: ../Sources/photo_sensor_hld.c
	@echo 'Building file: $<'
	@echo 'Executing target #35 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/photo_sensor_hld.args" -o "Sources/photo_sensor_hld_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/pit_init_c.obj: ../Sources/pit_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #36 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/pit_init.args" -o "Sources/pit_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/pot_hld_c.obj: ../Sources/pot_hld.c
	@echo 'Building file: $<'
	@echo 'Executing target #37 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/pot_hld.args" -o "Sources/pot_hld_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/rappid_func_s.obj: ../Sources/rappid_func.s
	@echo 'Building file: $<'
	@echo 'Executing target #38 $<'
	@echo 'Invoking: PowerPC Assembler'
	"$(PAToolsDirEnv)/mwasmeppc" @@"Sources/rappid_func.args" -o "Sources/rappid_func_s.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/rappid_utils_c.obj: ../Sources/rappid_utils.c
	@echo 'Building file: $<'
	@echo 'Executing target #39 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/rappid_utils.args" -o "Sources/rappid_utils_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/rchw_init_c.obj: ../Sources/rchw_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #40 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/rchw_init.args" -o "Sources/rchw_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/romcopy_c.obj: ../Sources/romcopy.c
	@echo 'Building file: $<'
	@echo 'Executing target #41 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/romcopy.args" -o "Sources/romcopy_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/rtc_init_c.obj: ../Sources/rtc_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #42 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/rtc_init.args" -o "Sources/rtc_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sbc_hld_c.obj: ../Sources/sbc_hld.c
	@echo 'Building file: $<'
	@echo 'Executing target #43 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/sbc_hld.args" -o "Sources/sbc_hld_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/school_zone_adc_c.obj: ../Sources/school_zone_adc.c
	@echo 'Building file: $<'
	@echo 'Executing target #44 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/school_zone_adc.args" -o "Sources/school_zone_adc_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/servo_motor_c.obj: ../Sources/servo_motor.c
	@echo 'Building file: $<'
	@echo 'Executing target #45 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/servo_motor.args" -o "Sources/servo_motor_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/siu_init_c.obj: ../Sources/siu_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #46 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/siu_init.args" -o "Sources/siu_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/slope_c.obj: ../Sources/slope.c
	@echo 'Building file: $<'
	@echo 'Executing target #47 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/slope.args" -o "Sources/slope_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sona_sensor_c.obj: ../Sources/sona_sensor.c
	@echo 'Building file: $<'
	@echo 'Executing target #48 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/sona_sensor.args" -o "Sources/sona_sensor_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/st7565_c.obj: ../Sources/st7565.c
	@echo 'Building file: $<'
	@echo 'Executing target #49 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/st7565.args" -o "Sources/st7565_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/stm_init_c.obj: ../Sources/stm_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #50 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/stm_init.args" -o "Sources/stm_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sw_handlers_s.obj: ../Sources/sw_handlers.s
	@echo 'Building file: $<'
	@echo 'Executing target #51 $<'
	@echo 'Invoking: PowerPC Assembler'
	"$(PAToolsDirEnv)/mwasmeppc" @@"Sources/sw_handlers.args" -o "Sources/sw_handlers_s.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/swt_init_c.obj: ../Sources/swt_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #52 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/swt_init.args" -o "Sources/swt_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sys_init_c.obj: ../Sources/sys_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #53 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/sys_init.args" -o "Sources/sys_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '

Sources/sysclk_init_c.obj: ../Sources/sysclk_init.c
	@echo 'Building file: $<'
	@echo 'Executing target #54 $<'
	@echo 'Invoking: PowerPC Compiler'
	"$(PAToolsDirEnv)/mwcceppc" @@"Sources/sysclk_init.args" -o "Sources/sysclk_init_c.obj" "$<" -MD -gccdep
	@echo 'Finished building: $<'
	@echo ' '


