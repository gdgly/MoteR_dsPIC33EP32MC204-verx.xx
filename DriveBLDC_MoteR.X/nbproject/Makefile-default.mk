#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../Application/Application.c ../Application/RampGenerator/RampGenerator.c ../Application/APP_uart.c ../Common/Delay/Delay.c ../Drivers/GPIO/GPIO.c ../Drivers/InputCapture/InputCapture.c ../Drivers/ADC/ADC.c ../Drivers/Timer/Timer.c ../Drivers/PWM/MCPWM.c ../Main/Main.c ../MotorControl/SpeedController/SpeedController.c ../MotorControl/Algorithm/svm.c ../MotorControl/CurrentController/CurrentController.c ../MotorControl/Braking/DCInjection.c ../MotorControl/CurrentController/CurrentLimit.c ../MotorControl/PIController/pi.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1693421727/Application.o ${OBJECTDIR}/_ext/1001158509/RampGenerator.o ${OBJECTDIR}/_ext/1693421727/APP_uart.o ${OBJECTDIR}/_ext/1444361294/Delay.o ${OBJECTDIR}/_ext/1185296862/GPIO.o ${OBJECTDIR}/_ext/1685861263/InputCapture.o ${OBJECTDIR}/_ext/1977904179/ADC.o ${OBJECTDIR}/_ext/1923288882/Timer.o ${OBJECTDIR}/_ext/1977889165/MCPWM.o ${OBJECTDIR}/_ext/761766712/Main.o ${OBJECTDIR}/_ext/1549342795/SpeedController.o ${OBJECTDIR}/_ext/629317769/svm.o ${OBJECTDIR}/_ext/1214161667/CurrentController.o ${OBJECTDIR}/_ext/654305744/DCInjection.o ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o ${OBJECTDIR}/_ext/1156753997/pi.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1693421727/Application.o.d ${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d ${OBJECTDIR}/_ext/1693421727/APP_uart.o.d ${OBJECTDIR}/_ext/1444361294/Delay.o.d ${OBJECTDIR}/_ext/1185296862/GPIO.o.d ${OBJECTDIR}/_ext/1685861263/InputCapture.o.d ${OBJECTDIR}/_ext/1977904179/ADC.o.d ${OBJECTDIR}/_ext/1923288882/Timer.o.d ${OBJECTDIR}/_ext/1977889165/MCPWM.o.d ${OBJECTDIR}/_ext/761766712/Main.o.d ${OBJECTDIR}/_ext/1549342795/SpeedController.o.d ${OBJECTDIR}/_ext/629317769/svm.o.d ${OBJECTDIR}/_ext/1214161667/CurrentController.o.d ${OBJECTDIR}/_ext/654305744/DCInjection.o.d ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d ${OBJECTDIR}/_ext/1156753997/pi.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1693421727/Application.o ${OBJECTDIR}/_ext/1001158509/RampGenerator.o ${OBJECTDIR}/_ext/1693421727/APP_uart.o ${OBJECTDIR}/_ext/1444361294/Delay.o ${OBJECTDIR}/_ext/1185296862/GPIO.o ${OBJECTDIR}/_ext/1685861263/InputCapture.o ${OBJECTDIR}/_ext/1977904179/ADC.o ${OBJECTDIR}/_ext/1923288882/Timer.o ${OBJECTDIR}/_ext/1977889165/MCPWM.o ${OBJECTDIR}/_ext/761766712/Main.o ${OBJECTDIR}/_ext/1549342795/SpeedController.o ${OBJECTDIR}/_ext/629317769/svm.o ${OBJECTDIR}/_ext/1214161667/CurrentController.o ${OBJECTDIR}/_ext/654305744/DCInjection.o ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o ${OBJECTDIR}/_ext/1156753997/pi.o

# Source Files
SOURCEFILES=../Application/Application.c ../Application/RampGenerator/RampGenerator.c ../Application/APP_uart.c ../Common/Delay/Delay.c ../Drivers/GPIO/GPIO.c ../Drivers/InputCapture/InputCapture.c ../Drivers/ADC/ADC.c ../Drivers/Timer/Timer.c ../Drivers/PWM/MCPWM.c ../Main/Main.c ../MotorControl/SpeedController/SpeedController.c ../MotorControl/Algorithm/svm.c ../MotorControl/CurrentController/CurrentController.c ../MotorControl/Braking/DCInjection.c ../MotorControl/CurrentController/CurrentLimit.c ../MotorControl/PIController/pi.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP128GM304
MP_LINKER_FILE_OPTION=,--script=p33EP128GM304.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1693421727/Application.o: ../Application/Application.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1693421727" 
	@${RM} ${OBJECTDIR}/_ext/1693421727/Application.o.d 
	@${RM} ${OBJECTDIR}/_ext/1693421727/Application.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/Application.c  -o ${OBJECTDIR}/_ext/1693421727/Application.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1693421727/Application.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1693421727/Application.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1001158509/RampGenerator.o: ../Application/RampGenerator/RampGenerator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1001158509" 
	@${RM} ${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1001158509/RampGenerator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/RampGenerator/RampGenerator.c  -o ${OBJECTDIR}/_ext/1001158509/RampGenerator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1693421727/APP_uart.o: ../Application/APP_uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1693421727" 
	@${RM} ${OBJECTDIR}/_ext/1693421727/APP_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1693421727/APP_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/APP_uart.c  -o ${OBJECTDIR}/_ext/1693421727/APP_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1693421727/APP_uart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1693421727/APP_uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1444361294/Delay.o: ../Common/Delay/Delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1444361294" 
	@${RM} ${OBJECTDIR}/_ext/1444361294/Delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1444361294/Delay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Common/Delay/Delay.c  -o ${OBJECTDIR}/_ext/1444361294/Delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1444361294/Delay.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1444361294/Delay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1185296862/GPIO.o: ../Drivers/GPIO/GPIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1185296862" 
	@${RM} ${OBJECTDIR}/_ext/1185296862/GPIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1185296862/GPIO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/GPIO/GPIO.c  -o ${OBJECTDIR}/_ext/1185296862/GPIO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1185296862/GPIO.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1185296862/GPIO.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1685861263/InputCapture.o: ../Drivers/InputCapture/InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1685861263" 
	@${RM} ${OBJECTDIR}/_ext/1685861263/InputCapture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1685861263/InputCapture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/InputCapture/InputCapture.c  -o ${OBJECTDIR}/_ext/1685861263/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1685861263/InputCapture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1685861263/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1977904179/ADC.o: ../Drivers/ADC/ADC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1977904179" 
	@${RM} ${OBJECTDIR}/_ext/1977904179/ADC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1977904179/ADC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/ADC/ADC.c  -o ${OBJECTDIR}/_ext/1977904179/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1977904179/ADC.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1977904179/ADC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1923288882/Timer.o: ../Drivers/Timer/Timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1923288882" 
	@${RM} ${OBJECTDIR}/_ext/1923288882/Timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1923288882/Timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/Timer/Timer.c  -o ${OBJECTDIR}/_ext/1923288882/Timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1923288882/Timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1923288882/Timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1977889165/MCPWM.o: ../Drivers/PWM/MCPWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1977889165" 
	@${RM} ${OBJECTDIR}/_ext/1977889165/MCPWM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1977889165/MCPWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/PWM/MCPWM.c  -o ${OBJECTDIR}/_ext/1977889165/MCPWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1977889165/MCPWM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1977889165/MCPWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761766712/Main.o: ../Main/Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761766712" 
	@${RM} ${OBJECTDIR}/_ext/761766712/Main.o.d 
	@${RM} ${OBJECTDIR}/_ext/761766712/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Main/Main.c  -o ${OBJECTDIR}/_ext/761766712/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761766712/Main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/761766712/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1549342795/SpeedController.o: ../MotorControl/SpeedController/SpeedController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1549342795" 
	@${RM} ${OBJECTDIR}/_ext/1549342795/SpeedController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1549342795/SpeedController.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/SpeedController/SpeedController.c  -o ${OBJECTDIR}/_ext/1549342795/SpeedController.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1549342795/SpeedController.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1549342795/SpeedController.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/629317769/svm.o: ../MotorControl/Algorithm/svm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/629317769" 
	@${RM} ${OBJECTDIR}/_ext/629317769/svm.o.d 
	@${RM} ${OBJECTDIR}/_ext/629317769/svm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/Algorithm/svm.c  -o ${OBJECTDIR}/_ext/629317769/svm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/629317769/svm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/629317769/svm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1214161667/CurrentController.o: ../MotorControl/CurrentController/CurrentController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1214161667" 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentController.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/CurrentController/CurrentController.c  -o ${OBJECTDIR}/_ext/1214161667/CurrentController.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1214161667/CurrentController.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1214161667/CurrentController.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/654305744/DCInjection.o: ../MotorControl/Braking/DCInjection.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/654305744" 
	@${RM} ${OBJECTDIR}/_ext/654305744/DCInjection.o.d 
	@${RM} ${OBJECTDIR}/_ext/654305744/DCInjection.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/Braking/DCInjection.c  -o ${OBJECTDIR}/_ext/654305744/DCInjection.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/654305744/DCInjection.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/654305744/DCInjection.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1214161667/CurrentLimit.o: ../MotorControl/CurrentController/CurrentLimit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1214161667" 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/CurrentController/CurrentLimit.c  -o ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1156753997/pi.o: ../MotorControl/PIController/pi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1156753997" 
	@${RM} ${OBJECTDIR}/_ext/1156753997/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1156753997/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/PIController/pi.c  -o ${OBJECTDIR}/_ext/1156753997/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1156753997/pi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1156753997/pi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1693421727/Application.o: ../Application/Application.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1693421727" 
	@${RM} ${OBJECTDIR}/_ext/1693421727/Application.o.d 
	@${RM} ${OBJECTDIR}/_ext/1693421727/Application.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/Application.c  -o ${OBJECTDIR}/_ext/1693421727/Application.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1693421727/Application.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1693421727/Application.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1001158509/RampGenerator.o: ../Application/RampGenerator/RampGenerator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1001158509" 
	@${RM} ${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d 
	@${RM} ${OBJECTDIR}/_ext/1001158509/RampGenerator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/RampGenerator/RampGenerator.c  -o ${OBJECTDIR}/_ext/1001158509/RampGenerator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1001158509/RampGenerator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1693421727/APP_uart.o: ../Application/APP_uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1693421727" 
	@${RM} ${OBJECTDIR}/_ext/1693421727/APP_uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1693421727/APP_uart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Application/APP_uart.c  -o ${OBJECTDIR}/_ext/1693421727/APP_uart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1693421727/APP_uart.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1693421727/APP_uart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1444361294/Delay.o: ../Common/Delay/Delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1444361294" 
	@${RM} ${OBJECTDIR}/_ext/1444361294/Delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1444361294/Delay.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Common/Delay/Delay.c  -o ${OBJECTDIR}/_ext/1444361294/Delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1444361294/Delay.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1444361294/Delay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1185296862/GPIO.o: ../Drivers/GPIO/GPIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1185296862" 
	@${RM} ${OBJECTDIR}/_ext/1185296862/GPIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1185296862/GPIO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/GPIO/GPIO.c  -o ${OBJECTDIR}/_ext/1185296862/GPIO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1185296862/GPIO.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1185296862/GPIO.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1685861263/InputCapture.o: ../Drivers/InputCapture/InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1685861263" 
	@${RM} ${OBJECTDIR}/_ext/1685861263/InputCapture.o.d 
	@${RM} ${OBJECTDIR}/_ext/1685861263/InputCapture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/InputCapture/InputCapture.c  -o ${OBJECTDIR}/_ext/1685861263/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1685861263/InputCapture.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1685861263/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1977904179/ADC.o: ../Drivers/ADC/ADC.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1977904179" 
	@${RM} ${OBJECTDIR}/_ext/1977904179/ADC.o.d 
	@${RM} ${OBJECTDIR}/_ext/1977904179/ADC.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/ADC/ADC.c  -o ${OBJECTDIR}/_ext/1977904179/ADC.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1977904179/ADC.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1977904179/ADC.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1923288882/Timer.o: ../Drivers/Timer/Timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1923288882" 
	@${RM} ${OBJECTDIR}/_ext/1923288882/Timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/1923288882/Timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/Timer/Timer.c  -o ${OBJECTDIR}/_ext/1923288882/Timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1923288882/Timer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1923288882/Timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1977889165/MCPWM.o: ../Drivers/PWM/MCPWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1977889165" 
	@${RM} ${OBJECTDIR}/_ext/1977889165/MCPWM.o.d 
	@${RM} ${OBJECTDIR}/_ext/1977889165/MCPWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Drivers/PWM/MCPWM.c  -o ${OBJECTDIR}/_ext/1977889165/MCPWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1977889165/MCPWM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1977889165/MCPWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/761766712/Main.o: ../Main/Main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/761766712" 
	@${RM} ${OBJECTDIR}/_ext/761766712/Main.o.d 
	@${RM} ${OBJECTDIR}/_ext/761766712/Main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../Main/Main.c  -o ${OBJECTDIR}/_ext/761766712/Main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/761766712/Main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/761766712/Main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1549342795/SpeedController.o: ../MotorControl/SpeedController/SpeedController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1549342795" 
	@${RM} ${OBJECTDIR}/_ext/1549342795/SpeedController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1549342795/SpeedController.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/SpeedController/SpeedController.c  -o ${OBJECTDIR}/_ext/1549342795/SpeedController.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1549342795/SpeedController.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1549342795/SpeedController.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/629317769/svm.o: ../MotorControl/Algorithm/svm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/629317769" 
	@${RM} ${OBJECTDIR}/_ext/629317769/svm.o.d 
	@${RM} ${OBJECTDIR}/_ext/629317769/svm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/Algorithm/svm.c  -o ${OBJECTDIR}/_ext/629317769/svm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/629317769/svm.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/629317769/svm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1214161667/CurrentController.o: ../MotorControl/CurrentController/CurrentController.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1214161667" 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentController.o.d 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentController.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/CurrentController/CurrentController.c  -o ${OBJECTDIR}/_ext/1214161667/CurrentController.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1214161667/CurrentController.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1214161667/CurrentController.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/654305744/DCInjection.o: ../MotorControl/Braking/DCInjection.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/654305744" 
	@${RM} ${OBJECTDIR}/_ext/654305744/DCInjection.o.d 
	@${RM} ${OBJECTDIR}/_ext/654305744/DCInjection.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/Braking/DCInjection.c  -o ${OBJECTDIR}/_ext/654305744/DCInjection.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/654305744/DCInjection.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/654305744/DCInjection.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1214161667/CurrentLimit.o: ../MotorControl/CurrentController/CurrentLimit.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1214161667" 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d 
	@${RM} ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/CurrentController/CurrentLimit.c  -o ${OBJECTDIR}/_ext/1214161667/CurrentLimit.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1214161667/CurrentLimit.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1156753997/pi.o: ../MotorControl/PIController/pi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1156753997" 
	@${RM} ${OBJECTDIR}/_ext/1156753997/pi.o.d 
	@${RM} ${OBJECTDIR}/_ext/1156753997/pi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../MotorControl/PIController/pi.c  -o ${OBJECTDIR}/_ext/1156753997/pi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1156753997/pi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -O0 -I".." -I"../Drivers/GPIO" -I"../Drivers/InputCapture" -I"../Application/RampGenerator" -I"../MotorControl/PIController" -I"../MotorControl/Algorithm" -I"../MotorControl/SpeedController" -I"../DMCI" -I"../Common/Delay" -I"../Common/Extern" -I"../Common/UserDefinition" -I"../Drivers/PWM" -I"../Middleware/ParameterDatabase" -I"../Middleware/CommunicationStack" -I"../MotorControl/Braking" -I"../MotorControl/CurrentController" -I"../Application" -msmart-io=1 -Wall -msfr-warn=off   -mno-eds-warn
	@${FIXDEPS} "${OBJECTDIR}/_ext/1156753997/pi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--no-force-link,--smart-io,-Map="${DISTDIR}/DriveBLDC750W.X.${IMAGE_TYPE}.map",--report-mem,--no-local-stack$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--no-force-link,--smart-io,-Map="${DISTDIR}/DriveBLDC750W.X.${IMAGE_TYPE}.map",--report-mem,--no-local-stack$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/DriveBLDC_MoteR.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
