################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
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
all: MotorControlSimulation.exe

# Tool invocations
MotorControlSimulation.exe: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MinGW C Linker'
	gcc  -o "MotorControlSimulation.exe" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) MotorControlSimulation.exe
	-@echo ' '

post-build:
	-@echo 'Run Scope'
	-D:\Tools\MSYS\usr\bin\bash.exe -c "cd 'D:/Projects/Simulation_Scripts/MCSim' && ./Debug/MotorControlSimulation.exe | D:/Tools/MSYS/ucrt64/bin/python3.exe ./scope.py"
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
