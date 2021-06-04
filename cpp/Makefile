# VPATH = src/adafruit/ src/main/ build/
vpath %.h src src/mjbots/moteus src/mjbots/pi3hat
vpath %.cpp src src/mjbots/moteus src/mjbots/pi3hat
vpath %.cc src src/mjbots/moteus src/mjbots/pi3hat
vpath %.o build
# VPATH = ./src/adafruit ./src/main ./build


SRC_DIR := ./src
BUILD_DIR := ./build
BIN_DIR := ./bin
COMMON_OBJS := pi3hat.o Adafruit_ADS1X15.o Adafruit_INA260.o I2CDevice.o
ADS_OBJS := ads_test.o $(COMMON_OBJS)
PI3T_OBJS := pi3hat_tool.o $(COMMON_OBJS)
DYNAMOMETER_OBJS := dynamometer.o $(COMMON_OBJS)

BINS := ads_test pi3hat_tool dynamometer

SYS_LIB_DIRS := /opt/vc/
SYS_INCLUDES := $(foreach lib,$(SYS_LIB_DIRS),$(addsuffix include,$(lib)))
SYS_OBJS := $(foreach lib,$(SYS_LIB_DIRS),$(addsuffix lib,$(lib)))

CFLAGS := -ggdb -c -pthread -fpermissive -I$(SYS_INCLUDES) -I src

all: $(BIN_DIR)/ads_test $(BIN_DIR)/pi3hat_tool

$(BIN_DIR)/ads_test : $(foreach obj,$(ADS_OBJS),$(addprefix $(BUILD_DIR)/,$(obj)))
	@echo "$@"
	g++ -ggdb $^ -o $@ -lbcm2835

$(BIN_DIR)/pi3hat_tool : $(foreach obj,$(PI3T_OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) 
	@echo "$@"
	g++ -ggdb $^ -o $@ -L$(SYS_OBJS) -lbcm_host 

$(BIN_DIR)/dynamometer : $(foreach obj,$(DYNAMOMETER_OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) 
	@echo "$@"
	g++ -ggdb $^ -o $@ -L$(SYS_OBJS) -I src/mjbots/moteus -lbcm_host -lbcm2835 -lpthread

$(BUILD_DIR)/%.o : %.cpp %.h
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o : %.cpp
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o : %.cc %.h
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o : %.cc
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@

# $(BUILD_DIR)/%.o : %.h
# 	@echo "    $@"
# 	g++ $(CFLAGS) $< -o $@ 

clean:
	rm -f build/* bin/*
