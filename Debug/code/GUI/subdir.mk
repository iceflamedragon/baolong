################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/GUI/GUI.c \
../code/GUI/application.c \
../code/GUI/display.c \
../code/GUI/port.c 

COMPILED_SRCS += \
./code/GUI/GUI.src \
./code/GUI/application.src \
./code/GUI/display.src \
./code/GUI/port.src 

C_DEPS += \
./code/GUI/GUI.d \
./code/GUI/application.d \
./code/GUI/display.d \
./code/GUI/port.d 

OBJS += \
./code/GUI/GUI.o \
./code/GUI/application.o \
./code/GUI/display.o \
./code/GUI/port.o 


# Each subdirectory must supply rules for building sources it contributes
code/GUI/%.src: ../code/GUI/%.c code/GUI/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -cs --dep-file="$(basename $@).d" --misrac-version=2004 -D__CPU__=tc26xb "-fC:/Users/lenovo/Desktop/ADS_Poject/ADS_Model2 - baidu - newborad/E05_pit_demo/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O2 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -Y0 -N0 -Z0 -o "$@" "$<" && \
	if [ -f "$(basename $@).d" ]; then sed.exe -r  -e 's/\b(.+\.o)\b/code\/GUI\/\1/g' -e 's/\\/\//g' -e 's/\/\//\//g' -e 's/"//g' -e 's/([a-zA-Z]:\/)/\L\1/g' -e 's/\d32:/@TARGET_DELIMITER@/g; s/\\\d32/@ESCAPED_SPACE@/g; s/\d32/\\\d32/g; s/@ESCAPED_SPACE@/\\\d32/g; s/@TARGET_DELIMITER@/\d32:/g' "$(basename $@).d" > "$(basename $@).d_sed" && cp "$(basename $@).d_sed" "$(basename $@).d" && rm -f "$(basename $@).d_sed" 2>/dev/null; else echo 'No dependency file to process';fi
	@echo 'Finished building: $<'
	@echo ' '

code/GUI/%.o: ./code/GUI/%.src code/GUI/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-code-2f-GUI

clean-code-2f-GUI:
	-$(RM) ./code/GUI/GUI.d ./code/GUI/GUI.o ./code/GUI/GUI.src ./code/GUI/application.d ./code/GUI/application.o ./code/GUI/application.src ./code/GUI/display.d ./code/GUI/display.o ./code/GUI/display.src ./code/GUI/port.d ./code/GUI/port.o ./code/GUI/port.src

.PHONY: clean-code-2f-GUI

