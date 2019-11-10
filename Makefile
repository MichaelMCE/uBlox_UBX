

CC=gcc
CP=g++


x64 := 0
ifeq ($(x64),1)
MDIR=x64
BITMODE=-m64
else
MDIR=x32
BITMODE=-m32
endif

# Graphite loop optimizations
GLOP = -ftree-vectorize -floop-interchange -floop-strip-mine -floop-block
#GLOP =



# -static-libgcc

#CFLAGS = $(BITMODE) -D_GNU_SOURCE  -I./libmylcd/include -Wall -march=k8 -mtune=k8 -Ofast -DSTBI_FAILURE_USERMSG -D_WIN32 -D_WIN32_WINNT=0x0501 -DWINVER=0x0501 -D__WIN32__=1 -std=gnu99 -funroll-loops -finline-functions -fomit-frame-pointer -pipe -s -DUSE_MMX -DHAVE_MMX -DHAVE_MMX1 -DUSE_MMX1 -mmmx -msse -mfpmath=sse,387 -fgcse-las -fgcse-sm -fgcse-lm -fmodulo-sched-allow-regmoves -fmodulo-sched -ftree-vectorizer-verbose=0 $(GLOP)
CFLAGS = $(BITMODE) -D_GNU_SOURCE -O2 -D_WIN32_WINNT=0x0601 -DWINVER=0x0601 -D__WIN32__=1 -std=gnu11 -Wall -march=i686 -mtune=i686
#CFLAGS = $(BITMODE) -g -D_GNU_SOURCE -Og -D_WIN32_WINNT=0x0601 -DWINVER=0x0601 -D__WIN32__=1 -std=gnu11 -Wall
CPPFLAGS = $(BITMODE) -D_GNU_SOURCE -O2 -I./libmylcd/include -Ofast -D_WIN32_WINNT=0x0601 -DWINVER=0x0601 -D__WIN32__=1 -std=c++11 -Wall -march=i686 -mtune=i686
LIBS = $(BITMODE) -D _GNU_SOURCE  -L"lib" -lm -lole32 -luuid -lwinmm



EXAMPLES=ubx.exe


all : $(EXAMPLES)



ubx.exe: ubx.o ubxcb.o
	$(CP) -o $@ $^ $(LIBS)
	strip.exe $@

%.o: %.c $(DEPS) 
	$(CC) -c -o $@ $< $(CFLAGS)

%.o: %.cpp $(DEPS) 
	$(CP) -c -o $@ $< $(CPPFLAGS)

clean :
	rm -f *.exe *.o *.bak


### PHONY define
.PHONY: all all-before all-after clean clean-custom

