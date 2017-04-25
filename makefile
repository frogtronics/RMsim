COMMON=-O2 -I../include -L../RMsim -std=c++11 -lstdc++

all:
	clang $(COMMON) sim.cpp   -lmujoco140 -lglfw.3 -o ../RMsim/sim
