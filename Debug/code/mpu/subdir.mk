################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/mpu/inv_mpu.c \
../code/mpu/inv_mpu_dmp_motion_driver.c \
../code/mpu/mpu6050.c \
../code/mpu/mpuiic.c 

COMPILED_SRCS += \
./code/mpu/inv_mpu.src \
./code/mpu/inv_mpu_dmp_motion_driver.src \
./code/mpu/mpu6050.src \
./code/mpu/mpuiic.src 

C_DEPS += \
./code/mpu/inv_mpu.d \
./code/mpu/inv_mpu_dmp_motion_driver.d \
./code/mpu/mpu6050.d \
./code/mpu/mpuiic.d 

OBJS += \
./code/mpu/inv_mpu.o \
./code/mpu/inv_mpu_dmp_motion_driver.o \
./code/mpu/mpu6050.o \
./code/mpu/mpuiic.o 


# Each subdirectory must supply rules for building sources it contributes
code/mpu/%.src: ../code/mpu/%.c code/mpu/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/lenovo/Desktop/ADS_Poject/ADS_Model1/E05_pit_demo/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/mpu\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/mpu/%.o: ./code/mpu/%.src code/mpu/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code-2f-mpu

clean-code-2f-mpu:
	-$(RM) ./code/mpu/inv_mpu.d ./code/mpu/inv_mpu.o ./code/mpu/inv_mpu.src ./code/mpu/inv_mpu_dmp_motion_driver.d ./code/mpu/inv_mpu_dmp_motion_driver.o ./code/mpu/inv_mpu_dmp_motion_driver.src ./code/mpu/mpu6050.d ./code/mpu/mpu6050.o ./code/mpu/mpu6050.src ./code/mpu/mpuiic.d ./code/mpu/mpuiic.o ./code/mpu/mpuiic.src

.PHONY: clean-code-2f-mpu

