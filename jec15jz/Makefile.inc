APPL_COBJS += balancer.o balancer_param.o

APPL_CXXOBJS += LogData.o Calibration.o Tail.o EV3RT.o Driver.o Section.o SectionData.o SysLog.o Calculation.o Stair.o StairDrive.o StairDriveData.o StairForce.o StairForceData.o StairTail.o StairTailData.o StairData.o Lookup.o LookupData.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION

# Include libraries
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile

endif
