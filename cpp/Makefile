# VPATH = src/adafruit/ src/main/ build/
vpath %.h src
vpath %.cpp src
vpath %.o build
# VPATH = ./src/adafruit ./src/main ./build

CFLAGS := -ggdb -c -pthread -fpermissive

SRC_DIR := ./src
BUILD_DIR := ./build
OBJS := Adafruit_ADS1X15.o Adafruit_INA260.o I2CDevice.o

all: ads_test

ads_test : ads_test.cpp $(OBJS)
	@echo "$@"
	g++ -ggdb $< $(foreach obj,$(OBJS),$(addprefix $(BUILD_DIR)/,$(obj))) -o $@ -lbcm2835


%.o : %.cpp %.h
	@echo "    $@"
	g++ $(CFLAGS) $< -o build/$@

# BUILD_DIR := build

# all: ads_test

# ads_test : src/main/ads_test.cpp build/Adafruit_ADS1X15.o build/Adafruit_INA260.o build/I2CDevice.o
# 	g++ -ggdb -o ads_test $^ -lbcm2835

# build/Adafruit_ADS1X15.o : src/adafruit/Adafruit_ADS1X15.cpp src/adafruit/Adafruit_ADS1X15.h build/I2CDevice.o
# 	g++ -ggdb -c -pthread $<

# build/Adafruit_INA260.o : src/adafruit/Adafruit_INA260.cpp src/adafruit/Adafruit_INA260.h build/I2CDevice.o
# 	g++ -ggdb -c -pthread $<

# build/I2CDevice.o : src/adafruit/I2CDevice.cpp src/adafruit/I2CDevice.h
# 	g++ -ggdb -c -pthread $< -fpermissive

# clean:
# 	rm -f build/*


# CC := g++
# LD := g++
# FLAGS := -fpermissive

# MODULES := adafruit main
# SRC_DIR := $(addprefix src/,$(MODULES))
# BUILD_DIR := $(addprefix build/,$(MODULES))

# SRC := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
# OBJ := $(patsubst src/%.cpp,build/%.o,$(SRC))
# INCLUDES := $(addprefix -I,$(SRC_DIR))

# vpath %.cpp $(SRC_DIR)

# define make-goal
# $1/%.o: %.cpp
# 	$(CC) $(INCLUDES) -c $$< -o $(FLAGS) $$@
# endef

# .PHONY: all checkdirs clean

# all: checkdirs build/ads_test

# build/ads_test: $(OBJ)
# 	$(LD) $^ -o $@


# checkdirs: $(BUILD_DIR)

# $(BUILD_DIR):
# 	@mkdir -p $@

# clean:
# 	@rm -rf $(BUILD_DIR)

# $(foreach bdir,$(BUILD_DIR),$(eval $(call make-goal,$(bdir))))


