################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/base/base.c \
../code/base/beep.c \
../code/base/encoder.c \
../code/base/imu660.c \
../code/base/inductance_adc.c \
../code/base/motor.c \
../code/base/mpu6050.c \
../code/base/tof.c 

COMPILED_SRCS += \
./code/base/base.src \
./code/base/beep.src \
./code/base/encoder.src \
./code/base/imu660.src \
./code/base/inductance_adc.src \
./code/base/motor.src \
./code/base/mpu6050.src \
./code/base/tof.src 

C_DEPS += \
./code/base/base.d \
./code/base/beep.d \
./code/base/encoder.d \
./code/base/imu660.d \
./code/base/inductance_adc.d \
./code/base/motor.d \
./code/base/mpu6050.d \
./code/base/tof.d 

OBJS += \
./code/base/base.o \
./code/base/beep.o \
./code/base/encoder.o \
./code/base/imu660.o \
./code/base/inductance_adc.o \
./code/base/motor.o \
./code/base/mpu6050.o \
./code/base/tof.o 


# Each subdirectory must supply rules for building sources it contributes
code/base/%.src: ../code/base/%.c code/base/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/lenovo/Desktop/ADS_Poject/ADS_Model2 - baidu - newborad/E05_pit_demo/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/base\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/base/%.o: ./code/base/%.src code/base/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code-2f-base

clean-code-2f-base:
	-$(RM) ./code/base/base.d ./code/base/base.o ./code/base/base.src ./code/base/beep.d ./code/base/beep.o ./code/base/beep.src ./code/base/encoder.d ./code/base/encoder.o ./code/base/encoder.src ./code/base/imu660.d ./code/base/imu660.o ./code/base/imu660.src ./code/base/inductance_adc.d ./code/base/inductance_adc.o ./code/base/inductance_adc.src ./code/base/motor.d ./code/base/motor.o ./code/base/motor.src ./code/base/mpu6050.d ./code/base/mpu6050.o ./code/base/mpu6050.src ./code/base/tof.d ./code/base/tof.o ./code/base/tof.src

.PHONY: clean-code-2f-base

