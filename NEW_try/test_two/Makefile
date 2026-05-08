CXX = g++
SRC = CST_1.cpp
OUT = cst_run

EPOS_INC = /home/e-wear/EPOS_Linux_Library/include

EPOS_LIB = /home/e-wear/EPOS_Linux_Library/lib/arm/v8

CXXFLAGS = -I$(EPOS_INC)
# link by absolute file names since generic symlinks aren't there
LDFLAGS  = $(EPOS_LIB)/libEposCmd.so.6.8.1.0 $(EPOS_LIB)/libftd2xx.so.1.4.8 -Wl,-rpath,$(EPOS_LIB)

all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(SRC) $(CXXFLAGS) $(LDFLAGS) -o $(OUT)

clean:
	rm -f $(OUT)

