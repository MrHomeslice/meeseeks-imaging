CC = g++
TARGET_EXEC ?= v1

BUILD_DIR ?= ./build
TARGET_DIR ?= ./bin
SRC_DIRS ?= .

SRCS := $(shell find -name '*.cpp' -not -path './bin/*' -printf '%f\n')
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -name '*.h' -printf '%h\n' | sort -u)
INC_FLAGS1 := $(addprefix -I,$(INC_DIRS))
LIB_PATH=/<teamnumber>/ntcore/
INC_FLAGS_NTCORE := -I/home/ubuntu/Documents/frc/ntcore/include -I/home/ubuntu/Documents/frc/ntcore/wpiutil/include
BIN_FLAGS := ''

CPPFLAGS ?= $(INC_FLAGS_NTCORE) -O3 -std=c++11 `pkg-config --cflags opencv`
LDFLAGS_NTCORE ?= -L/home/ubuntu/Documents/frc/ntcore -lntcore -lwpiutil -lpthread `pkg-config --libs opencv`
LDFLAGS ?= -L/home/ubuntu/Documents/frc/ntcore -lntcore -lwpiutil -lpthread `pkg-config --libs opencv`

$(TARGET_DIR)/$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean

clean:
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p
