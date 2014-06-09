# This makefile is written for gcc running under Solaris on a SPARCstation.
# To compile VPR on other systems, you may have to change:
# (1) CC to the name of your C compiler.
# (2) LIB_DIR to point at your directory of X11 libraries (libX11.a, etc.)
# (3) On many systems, the -R/usr/openwin/lib option on the LIB line
#     will have to be removed.
# (4) X11_INCLUDE to point at the directory containing "x11/xlib.h" etc.
# (5) OPT_FLAGS should be changed to whatever options turn on maximum
#     optimization in your compiler.
# (6) If your system does not support X11 (e.g. you're running on Windows NT)
#     then add -DNO_GRAPHICS to the end of the "FLAGS =" line.
CC = gcc

#SunOS lines below.
#LIB_DIR = -L/usr/lib/X11R5
#LIB = -static -lX11 -lm
#X11_INCLUDE = -I/usr/include

# On many non-Solaris machines, use LIB_DIR = -L/usr/lib/X11R5

LIB_DIR = -L/usr/openwin/lib
SRC_DIR = .
# Sometimes get errors under Solaris if you don't use the -R option
# to (I think) put info about where the shared object libraries are
# right into the binary.  Shouldn't be necessary, but it is on our machines.

#LIB = -lX11 -lm -R/usr/openwin/lib
LIB = -lX11 -lm -pthread

#X11_INCLUDE = -I/usr/openwin/include

# Overly strict flags line below.  Lots of useless warnings, but can
# let you look for redudant declarations.
# To avoid redundant declarations here I use -D__STDC instead of
# -D__USE_FIXED_PROTOTYPES, but that means some prototypes are missing.

#FLAGS = -Wall -Wpointer-arith -Wcast-qual -Wstrict-prototypes -O -D__STDC__ -ansi -pedantic -Wredundant-decls -Wmissing-prototypes -Wshadow -Wcast-align -D_POSIX_SOURCE

#Flags to be passed to the compiler.  First is for strict warnings,
#second for interactive debugging and third for optimization.

#-D_POSIX_SOURCE stops extra declarations from being included in math.h
#and causing -Wshadow to complain about conflicts with y1 in math.h
#(Bessel function 1 of the second kind)
WARN_FLAGS = -Wall -Wpointer-arith -Wcast-qual -Wstrict-prototypes -O -D__USE_FIXED_PROTOTYPES__ -ansi -pedantic -Wmissing-prototypes -Wshadow -Wcast-align -D_POSIX_SOURCE
DEBUG_FLAGS = -g -pg -Wall
OPT_FLAGS = -fPIC -O3 -pthread

#I found that in Ubuntu12.04-x86_64 and gcc4,6, the P&R result was same whether or not added -std=c99,
#but they were different in Fedora17-i386 system, gcc4.7.3
#FLAGS = $(OPT_FLAGS)
FLAGS = $(DEBUG_FLAGS) $(OPT_FLAGS)

#Uncomment line below if X Windows isn't installed on your system.
#FLAGS = $(OPT_FLAGS) -DNO_GRAPHICS

#Useful flags on HP machines
#DEBUG_FLAGS = -Aa -g
#OPT_FLAGS = -Aa +O3

EXE = ../bin/vpr

#in obj file, it didn't had test_h.o and graphics.o, it should compile seperately
OBJ = ../obj/main.o ../obj/util.o ../obj/read_netlist.o ../obj/print_netlist.o ../obj/check_netlist.o \
	  ../obj/read_arch.o ../obj/place_and_route.o ../obj/place.o ../obj/route_common.o ../obj/route_timing.o \
	  ../obj/route_tree_timing.o ../obj/route_breadth_first.o ../obj/draw.o ../obj/graphics.o ../obj/stats.o \
	  ../obj/segment_stats.o ../obj/rr_graph.o ../obj/rr_graph2.o ../obj/rr_graph_sbox.o ../obj/rr_graph_util.o \
      ../obj/rr_graph_timing_params.o ../obj/rr_graph_indexed_data.o ../obj/rr_graph_area.o ../obj/check_rr_graph.o \
      ../obj/check_route.o ../obj/hash.o ../obj/heapsort.o ../obj/read_place.o ../obj/net_delay.o \
	  ../obj/path_delay.o ../obj/path_delay2.o ../obj/vpr_utils.o ../obj/timing_place_lookup.o \
	  ../obj/timing_place.o ../obj/place_parallel.o ../obj/path_delay_parallel.o

#attention, the test_h.c and graphics.c should delete from SRC, it should compile seperately.
SRC = main.c util.c read_netlist.c print_netlist.c check_netlist.c read_arch.c \
	  place_and_route.c place.c route_common.c route_timing.c route_tree_timing.c \
	  route_breadth_first.c draw.c graphics.c stats.c segment_stats.c rr_graph.c \
	  rr_graph2.c rr_graph_sbox.c rr_graph_util.c rr_graph_timing_params.c \
	  rr_graph_indexed_data.c rr_graph_area.c check_rr_graph.c check_route.c hash.c \
	  heapsort.c read_place.c net_delay.c path_delay.c path_delay2.c \
	  vpr_utils.c timing_place_lookup.c timing_place.c place_parallel.c path_delay_parallel.c

Header = util.h vpr_types.h globals.h graphics.h read_netlist.h \
         print_netlist.h check_netlist.h read_arch.h stats.h \
		 segment_stats.h draw.h place_and_route.h place.h route_export.h \
		 route_common.h route_timing.h route_tree_timing.h route_breadth_first.h \
		 rr_graph.h rr_graph2.h rr_graph_sbox.h rr_graph_util.h \
		 rr_graph_timing_params.h rr_graph_indexed_data.h rr_graph_area.h \
		 check_rr_graph.h check_route.h hash.h heapsort.h read_place.h \
         path_delay.h path_delay2.h net_delay.h vpr_utils.h \
		 timing_place_lookup.h timing_place.h place_parallel.h path_delay_parallel.h

# Add purify before $(CC) in the link line below to run purify on VPR.

#link all needed object files to binary file.
#FIXME: the statement must define at all object files, graphics.o and test_h.o,
#otherwise, it will not compile all the needed object files.
$(EXE): $(OBJ) ../obj/test_h.o
	$(CC) $(FLAGS) $(OBJ) ../obj/test_h.o -o $(EXE) $(LIB_DIR) $(LIB)

# test_h.c is a dummy file -- it just checks the header files against each
# other to see if any global variable definitions conflict in the various
# header files.
../obj/test_h.o: $(SRC_DIR)/test_h.c $(Header)
	$(CC) -c $(WARN_FLAGS) -Wredundant-decls $(SRC_DIR)/test_h.c -o ../obj/test_h.o

#then I should set most object files. Attention, I should use %.o, %.c, not *.o, *.c.
../obj/%.o : $(SRC_DIR)/%.c
	$(CC) $(FLAGS) -c $< -o $@

clean:
	rm -fv ../obj/*.o $(EXE)
	echo "remove all object files and vpr."
