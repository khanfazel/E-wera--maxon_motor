CXX = g++
SRC = ff_pd.cpp
OUT = ff_run

EPOS_INC = /home/e-wear/EPOS_Linux_Library/include
EPOS_LIB = /home/e-wear/EPOS_Linux_Library/lib/arm/v8

CXXFLAGS = -std=c++17 -Wall -O2 -I$(EPOS_INC)
LDFLAGS  = -L$(EPOS_LIB) -lEposCmd -Wl,-rpath,$(EPOS_LIB)

all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(OUT)

run: $(OUT)
	sudo env LD_LIBRARY_PATH=$(EPOS_LIB) ./$(OUT)

clean:
	rm -f $(OUT)
