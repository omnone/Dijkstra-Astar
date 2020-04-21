LEDAROOT := /usr/local/LEDA
BOOSTROOT := /usr/local/include/
SRCDIR = src

TARGET := main
OBJS   := $(SRCDIR)/main.cpp
HEADERS := $(SRCDIR)/dijkstra_sp.cpp $(SRCDIR)/a_star.cpp

CXX := g++ -w

#CXXFLAGS := -O3
CXXFLAGS := $(CXXFLAGS) -I  $(BOOSTROOT) -I $(LEDAROOT)/incl
LIBS     += -L $(LEDAROOT) -L/usrX11R6/lib64/ -lm -lX11 -lleda

all: run

compile: $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) $(HEADERS) $(LIBS) -o $(TARGET)
  
run:  main
	./main
   
clean: rm -rf $(TARGET) 