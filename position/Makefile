# Compiler
CXX = g++

# Source and output
SRC = pd.cpp
OUT = pd_run

# ---- EPOS Library Paths (CHANGE if needed) ----
EPOS_INC = /home/e-wear/EPOS_Linux_Library/include
EPOS_LIB = /home/e-wear/EPOS_Linux_Library/lib/arm/v8

# Flags
CXXFLAGS = -std=c++17 -Wall -I$(EPOS_INC)
LDFLAGS  = -L$(EPOS_LIB) -lEposCmd -Wl,-rpath,$(EPOS_LIB)

# Build
all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(OUT)

# Clean
clean:
	rm -f $(OUT)
