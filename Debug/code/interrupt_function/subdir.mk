################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/interrupt_function/timer1_pwm.c \
../code/interrupt_function/timer2.c \
../code/interrupt_function/timer3_key.c \
../code/interrupt_function/timer4_device.c 

COMPILED_SRCS += \
./code/interrupt_function/timer1_pwm.src \
./code/interrupt_function/timer2.src \
./code/interrupt_function/timer3_key.src \
./code/interrupt_function/timer4_device.src 

C_DEPS += \
./code/interrupt_function/timer1_pwm.d \
./code/interrupt_function/timer2.d \
./code/interrupt_function/timer3_key.d \
./code/interrupt_function/timer4_device.d 

OBJS += \
./code/interrupt_function/timer1_pwm.o \
./code/interrupt_function/timer2.o \
./code/interrupt_function/timer3_key.o \
./code/interrupt_function/timer4_device.o 


# Each subdirectory must supply rules for building sources it contributes
code/interrupt_function/%.src: ../code/interrupt_function/%.c code/interrupt_function/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/lenovo/Desktop/ADS_Poject/ADS_Model2 - baidu - newborad/E05_pit_demo/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/interrupt_function\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/interrupt_function/%.o: ./code/interrupt_function/%.src code/interrupt_function/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code-2f-interrupt_function

clean-code-2f-interrupt_function:
	-$(RM) ./code/interrupt_function/timer1_pwm.d ./code/interrupt_function/timer1_pwm.o ./code/interrupt_function/timer1_pwm.src ./code/interrupt_function/timer2.d ./code/interrupt_function/timer2.o ./code/interrupt_function/timer2.src ./code/interrupt_function/timer3_key.d ./code/interrupt_function/timer3_key.o ./code/interrupt_function/timer3_key.src ./code/interrupt_function/timer4_device.d ./code/interrupt_function/timer4_device.o ./code/interrupt_function/timer4_device.src

.PHONY: clean-code-2f-interrupt_function

