# VPATH = src/adafruit/ src/main/ build/
vpath %.h src src/mjbots/moteus src/mjbots/pi3hat
vpath %.cpp src src/mjbots/moteus src/mjbots/pi3hat
vpath %.cc src src/mjbots/moteus src/mjbots/pi3hat
vpath %.o build
# VPATH = ./src/adafruit ./src/main ./build

CFLAGS := -ggdb -c -pthread -fpermissive

SRC_DIR := ./src
BUILD_DIR := ./build
BIN_DIR := ./bin
OBJS := ads_test.o Adafruit_ADS1X15.o Adafruit_INA260.o I2CDevice.o
PI3T_OBJS := pi3hat_tool.o pi3hat.o 

SYS_LIB_DIRS := /opt/vc/
SYS_INCLUDES := $(foreach lib,$(SYS_LIB_DIRS),$(addsuffix include,$(lib)))
SYS_OBJS := $(foreach lib,$(SYS_LIB_DIRS),$(addsuffix lib,$(lib)))

all: $(BIN_DIR)/ads_test $(BIN_DIR)/pi3hat_tool

$(BIN_DIR)/ads_test : $(foreach obj,$(OBJS),$(addprefix $(BUILD_DIR)/,$(obj)))
	@echo "$@"
# g++ -ggdb $(foreach obj,$(OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) -o $@ -lbcm2835
	g++ -ggdb $^ -o $@ -lbcm2835

$(BIN_DIR)/pi3hat_tool : $(foreach obj,$(PI3T_OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) 
	@echo "$@"
# g++ -ggdb $(foreach obj,$(PI3T_OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) -o $@
	g++ -ggdb $^ -o $@ -lbcm_host -L$(SYS_OBJS)

$(BUILD_DIR)/%.o : %.cpp %.h
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@ -I$(SYS_INCLUDES)

$(BUILD_DIR)/%.o : %.cpp
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@ -I$(SYS_INCLUDES)

$(BUILD_DIR)/%.o : %.cc %.h
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@ -I$(SYS_INCLUDES)

$(BUILD_DIR)/%.o : %.cc
	@echo "    $@"
	g++ $(CFLAGS) $< -o $@ -I$(SYS_INCLUDES)

clean:
	rm -f build/* bin/*
